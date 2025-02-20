/**
 *****************************************************************************************
 *
 * @file dfu_port.c
 *
 * @brief  DFU port Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

 /*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "dfu_port.h"
#include "hal_flash.h"
#include "grx_hal.h"
#include "otas.h"
#include "grx_sys.h"
#include "ring_buffer.h"
#include "flash_scatter_config.h"
#ifdef ENABLE_DFU_SPI_FLASH
    #include "gr55xx_spi_flash.h"
#endif


#define DFU_BUFFER_SIZE                 2048                                                         /**< The dfu buffer size. */
#define FLASH_CACHE_BUFFER_SIZE         4096
#define ONCE_WRITE_DATA_LEN             1024                                                          /**< The data length of flash write-once. */

#define MIN_RING_BUFFER_SIZE                (DFU_BUFFER_SIZE + FLASH_CACHE_BUFFER_SIZE - ONCE_WRITE_DATA_LEN)
#if DFU_BUFFER_SIZE < 2*1024
#error "Error: DFU_BUFFER_SIZE < 2KB"
#endif
#if FLASH_CACHE_BUFFER_SIZE < 4*1024
#error "Error: FLASH_CACHE_BUFFER_SIZE < 4KB"
#endif
#if MIN_RING_BUFFER_SIZE < 2*1024
#error "Error: MIN_RING_BUFFER_SIZE < 2KB"
#endif

#if defined(SOC_GR533X) || defined(SOC_GR5405)
#define ENV_BUFFER_SIZE                 400
#define ENV_CHECK_SUM_OFFSET            0x28
#define ENV_PAGE_START_ADDR_OFFSET      0x24
#endif


#define FAST_DFU_INIT_STATE             0x00
#define FAST_DFU_ERASE_FLASH_STATE      0x01
#define FAST_DFU_PROGRAM_FLASH_STATE    0x02

enum
{
    DFU_ACK_SUCCESS = 0x01,
    DFU_ACK_ERROR,
};

enum
{
    DFU_FLASH_INNER,
    DFU_FLASH_SPI = 0x01,

    FAST_DFU_INNER = 0x02,
    FAST_DFU_SPI = 0X03,
};

enum
{
    NORMAL_FIRMWARE = 0x00,
    SIGN_FIRMWARE = 0x01,
    SECURITY_SIGN_FIRMWARE = 0x02,
};

enum
{
    DFU_ERASE_FLASH_ERROR = 0x00,
    DFU_ERASE_FLASH_START,
    DFU_ERASING_FLASH,
    DFU_ERASE_FLASH_END,
    DFU_ERASE_FLASH_OVERLAP,
    DFU_ERASE_FLASH_OPER_FAILED,
    DFU_ERASE_EXT_FLASH_NOT_EXIST,
};

#if defined(SOC_GR533X) || defined(SOC_GR5405)
extern uint32_t         dfu_env_all;
#else
extern uint32_t         all_check_sum;
extern uint32_t         page_start_addr;
#endif

extern uint8_t          cmd_receive_flag;

extern void dfu_programing(uint16_t len);
extern void dfu_flash_type_set(uint8_t flash_type);
extern uint32_t dfu_flash_write(const uint32_t addr,const uint8_t *buf, const uint32_t size);
extern uint32_t dfu_flash_read(const uint32_t addr,const uint8_t *buf, const uint32_t size);
extern bool dfu_flash_erase(const uint32_t addr, const uint32_t size);
extern void normal_dfu_schedule(void);
extern uint16_t get_conn_hdl_by_idx(uint16_t conn_idx);

#ifdef SOC_GR5526
extern void send_frame_state(dfu_receive_frame_t *p_frame, uint8_t state);
extern uint32_t dfu_flash_cal_check_sum(uint32_t start_addr, uint16_t len);
extern uint32_t dfu_flash_programe(uint32_t address, uint8_t *p_write_buf, uint16_t write_len);

#if (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)
    static bool portable_flash_erase(const uint32_t addr, const uint32_t size);
    static bool portable_flash_erase_chip(void);
#endif

#endif


#if defined(SOC_GR533X) || defined(SOC_GR5405)
extern void dfu_programing_start(uint32_t all_size);
extern void dfu_programing_end(uint8_t status);
extern void dfu_frame_send(uint8_t *data,uint16_t len,uint16_t cmd_type);
static void dfu_program_start(uint32_t all_size);
#else
extern void dfu_program_start(uint32_t all_size);
extern void dfu_program_end(uint8_t status);
extern bool dfu_security_check_enable(void);
extern void dfu_send_frame(uint8_t *data,uint16_t len,uint16_t cmd_type);
#endif

static void ble_send_data(uint8_t *p_data, uint16_t length);

#ifdef ENABLE_DFU_CUSTOM_BUFFER
static uint8_t * s_p_cmd_buffer         = NULL;
#if defined(SOC_GR5526) || defined(SOC_GR5X25) || defined(SOC_GR533X)
static uint8_t * s_p_flash_cache_buffer = NULL;
#endif
static uint8_t * s_p_ring_buffer        = NULL;
static uint16_t  s_ring_buffer_size     = 0;
static uint8_t * s_p_fast_cache_buffer  = NULL;
#else
static uint8_t                 __s_buffer_pool[DFU_BUFFER_SIZE + FLASH_CACHE_BUFFER_SIZE];
#define s_p_cmd_buffer         __s_buffer_pool
#define s_p_flash_cache_buffer (__s_buffer_pool + DFU_BUFFER_SIZE)
#define s_p_ring_buffer        __s_buffer_pool
#define s_ring_buffer_size     MIN_RING_BUFFER_SIZE
#define s_p_fast_cache_buffer  (__s_buffer_pool + s_ring_buffer_size)
#endif

static dfu_info_t           s_dfu_info;
static ring_buffer_t        s_ble_rx_ring_buffer;
static bool                 s_ring_buffer_over_flag = false;
static uint8_t              s_fast_dfu_state  = FAST_DFU_INIT_STATE;
static bool                 s_program_end_flag = false;
static uint32_t             s_program_address = 0x00;
static uint32_t             s_file_size       = 0x00;
static uint16_t             s_erase_all_count = 0;
static uint32_t             s_all_write_size  = 0x00;
static uint16_t             s_erase_count = 0;
static uint8_t              s_fast_dfu_mode    = 0x00;
static dfu_image_info_t     s_now_img_info;
static dfu_enter_callback   s_dfu_enter_func = NULL;

static uint8_t              s_ota_conn_index = BLE_GAP_INVALID_CONN_INDEX;

#if defined(SOC_GR533X) || defined(SOC_GR5405)
static uint32_t     page_start_addr;
static uint32_t     *p_page_start_addr = NULL;
static uint32_t     all_check_sum;
static uint32_t     *p_all_check_sum = NULL;
static uint8_t      env_data_buffer[ENV_BUFFER_SIZE] = {0};

static dfu_func_t       s_dfu_func  = 
{
    .dfu_ble_send_data      = ble_send_data,
    .dfu_flash_read         = hal_exflash_read,
    .dfu_flash_write        = hal_exflash_write,
    .dfu_flash_erase        = hal_exflash_erase,
    .dfu_flash_get_info     = hal_flash_get_info,
    .dfu_flash_feat_enable  = NULL,
};

static dfu_buf_table_t dfu_buffer =
{
    #ifdef ENABLE_DFU_CUSTOM_BUFFER
    0
    #else
    .frame_buf  = s_p_cmd_buffer,
    .frame_size = DFU_BUFFER_SIZE,
    .jlink_buf  = NULL,
    .jlink_size = 0,
    .cache_buf  = s_p_flash_cache_buffer,
    .cache_size = FLASH_CACHE_BUFFER_SIZE,
    .env_buf    = env_data_buffer,
    .env_size = ENV_BUFFER_SIZE,
    #endif
};
#else
static bool          s_flash_security_status = false;
static dfu_func_t    s_dfu_func =
{
    .dfu_ble_send_data      = ble_send_data,
    .dfu_uart_send_data     = NULL,
#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)
    .dfu_flash_read         = dfu_portable_flash_read,
    .dfu_flash_write        = dfu_portable_flash_write,
    .dfu_flash_erase        = portable_flash_erase,
    .dfu_flash_erase_chip   = portable_flash_erase_chip,
#else
    .dfu_flash_read         = hal_flash_read,
    .dfu_flash_write        = hal_flash_write,
    .dfu_flash_erase        = hal_flash_erase,
    .dfu_flash_erase_chip   = hal_flash_erase_chip,
#endif
    .dfu_flash_set_security = hal_flash_set_security,
    .dfu_flash_get_security = hal_flash_get_security,
    .dfu_flash_get_info     = hal_flash_get_info,
};
#endif

#if defined(SOC_GR5526) || defined(SOC_GR5X25)

static dfu_buffer_t         dfu_buffer =
{
    #ifdef ENABLE_DFU_CUSTOM_BUFFER
    0
    #else
    .rec_data_buffer      = s_p_cmd_buffer,
    .rec_data_buffer_size = DFU_BUFFER_SIZE,
    .flash_op_buffer      = s_p_flash_cache_buffer,
    .flash_op_buffer_size = FLASH_CACHE_BUFFER_SIZE
    #endif
};
#endif


#ifdef ENABLE_DFU_SPI_FLASH
static void dfu_spi_flash_init(uint8_t* p_data);                                            /**< flash init. */
static uint32_t dfu_spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes);     /**< read flash data. */
static uint32_t dfu_spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes);    /**< write flash data. */
static void dfu_spi_flash_device_info(uint32_t *id, uint32_t *size);                        /**< get flash device information. */

#if defined(SOC_GR533X) || defined(SOC_GR5405)
static bool dfu_spi_flash_erase(uint32_t erase_type, uint32_t addr, uint32_t size);         /**< erase flash. */

static dfu_spi_flash_func_t s_dfu_spi_flash_func=                                           /**< SPI used functions config definition. */
{
    .dfu_spi_flash_init       = dfu_spi_flash_init,
    .dfu_spi_flash_read       = dfu_spi_flash_read,
    .dfu_spi_flash_write      = dfu_spi_flash_write,
    .dfu_spi_flash_erase      = dfu_spi_flash_erase,
    .dfu_spi_flash_get_info   = dfu_spi_flash_device_info,
    .dfu_flash_feat_enable    = NULL,
};
#else
static bool dfu_spi_flash_sector_erase(uint32_t address, uint32_t size);                    /**< erase flash sector. */
static bool dfu_spi_flash_chip_erase(void);                                                 /**< erase flash chip. */

static dfu_spi_flash_func_t s_dfu_spi_flash_func=                                           /**< SPI used functions config definition. */
{
    .dfu_spi_flash_init       = dfu_spi_flash_init,
    .dfu_spi_flash_read       = dfu_spi_flash_read,
    .dfu_spi_flash_write      = dfu_spi_flash_write,
    .dfu_spi_flash_erase      = dfu_spi_flash_sector_erase,
    .dfu_spi_flash_erase_chip = dfu_spi_flash_chip_erase,
    .dfu_spi_flash_get_info   = dfu_spi_flash_device_info,
};
#endif

#endif


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
#if defined(SOC_GR533X) || defined(SOC_GR5405)
static void dfu_program_start(uint32_t all_size)
{
    dfu_programing_start(all_size);
}

static void dfu_program_end(uint8_t status)
{
    dfu_programing_end(status);
}

static void dfu_send_frame(uint8_t *data,uint16_t len,uint16_t cmd_type)
{
    dfu_frame_send(data, len, cmd_type);
}
#endif


#ifdef ENABLE_DFU_SPI_FLASH
static void dfu_spi_flash_init(uint8_t *p_data)
{
    flash_init_t flash_init;
    flash_io_t *flash_io = (flash_io_t*)&flash_init.flash_io;

#ifdef SOC_GR5515
    uint8_t flash_type = p_data[0];
    const app_io_type_t gpio_type[]   = {APP_IO_TYPE_NORMAL,APP_IO_TYPE_AON,APP_IO_TYPE_MSIO};
    const uint32_t      gpio_pin[]    = {APP_IO_PIN_0,APP_IO_PIN_1,APP_IO_PIN_2,APP_IO_PIN_3,APP_IO_PIN_4,APP_IO_PIN_5,APP_IO_PIN_6,APP_IO_PIN_7,\
                                         APP_IO_PIN_8,APP_IO_PIN_9,APP_IO_PIN_10,APP_IO_PIN_11,APP_IO_PIN_12,APP_IO_PIN_13,APP_IO_PIN_14,APP_IO_PIN_15,\
                                         APP_IO_PIN_16,APP_IO_PIN_17,APP_IO_PIN_18,APP_IO_PIN_19,APP_IO_PIN_20,APP_IO_PIN_21,APP_IO_PIN_22,APP_IO_PIN_23,\
                                         APP_IO_PIN_24,APP_IO_PIN_25,APP_IO_PIN_26,APP_IO_PIN_27,APP_IO_PIN_28,APP_IO_PIN_29,APP_IO_PIN_30,APP_IO_PIN_31,};
    const app_io_mux_t gpio_pin_mux[] = {APP_IO_MUX_0,APP_IO_MUX_1,APP_IO_MUX_2,APP_IO_MUX_3,APP_IO_MUX_4,APP_IO_MUX_5,APP_IO_MUX_6,APP_IO_MUX_7,APP_IO_MUX_8};

    if (flash_type ==  0x01)
    {
        //SPI flash
        flash_io->spi_cs.gpio            = gpio_type[p_data[1]];
        flash_io->spi_cs.pin             = gpio_pin[p_data[2]];
        flash_io->spi_cs.mux             = gpio_pin_mux[p_data[3]];
        flash_io->spi_clk.gpio           = gpio_type[p_data[4]];
        flash_io->spi_clk.pin            = gpio_pin[p_data[5]];
        flash_io->spi_clk.mux            = gpio_pin_mux[p_data[6]];
        flash_io->spi_io0.spim_mosi.gpio = gpio_type[p_data[7]];
        flash_io->spi_io0.spim_mosi.pin  = gpio_pin[p_data[8]];
        flash_io->spi_io0.spim_mosi.mux  = gpio_pin_mux[p_data[9]];
        flash_io->spi_io1.spim_miso.gpio = gpio_type[p_data[10]];
        flash_io->spi_io1.spim_miso.pin  = gpio_pin[p_data[11]];
        flash_io->spi_io1.spim_miso.mux  = gpio_pin_mux[p_data[12]];

        flash_init.spi_type = FLASH_SPIM_ID;
        flash_init.is_dual_line = false;
        flash_init.is_high_freq = false;

        spi_flash_init(&flash_init);
    }
    else if (flash_type ==  0x02)//QSPI flash
    {
        flash_io->spi_cs.gpio           = gpio_type[p_data[1]];
        flash_io->spi_cs.pin            = gpio_pin[p_data[2]];
        flash_io->spi_cs.mux            = gpio_pin_mux[p_data[3]];
        flash_io->spi_clk.gpio          = gpio_type[p_data[4]];
        flash_io->spi_clk.pin           = gpio_pin[p_data[5]];
        flash_io->spi_clk.mux           = gpio_pin_mux[p_data[6]];
        flash_io->spi_io0.qspi_io0.gpio = gpio_type[p_data[7]];
        flash_io->spi_io0.qspi_io0.pin  = gpio_pin[p_data[8]];
        flash_io->spi_io0.qspi_io0.mux  = gpio_pin_mux[p_data[9]];
        flash_io->spi_io1.qspi_io1.gpio = gpio_type[p_data[10]];
        flash_io->spi_io1.qspi_io1.pin  = gpio_pin[p_data[11]];
        flash_io->spi_io1.qspi_io1.mux  = gpio_pin_mux[p_data[12]];
        flash_io->qspi_io2.gpio         = gpio_type[p_data[13]];
        flash_io->qspi_io2.pin          = gpio_pin[p_data[14]];
        flash_io->qspi_io2.mux          = gpio_pin_mux[p_data[15]];
        flash_io->qspi_io3.gpio         = gpio_type[p_data[16]];
        flash_io->qspi_io3.pin          = gpio_pin[p_data[17]];
        flash_io->qspi_io3.mux          = gpio_pin_mux[p_data[18]];

        flash_init.spi_type = p_data[19] ? FLASH_QSPI_ID1 : FLASH_QSPI_ID0;

        if(p_data[19] & 0x40)
        {
            flash_init.is_dual_line = true;
        }
        else
        {
            flash_init.is_dual_line = false;
        }

        flash_init.is_high_freq = false;

        spi_flash_init(&flash_init);
    }
    else
    {
        //Unkown flash type
    }
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR5526)
    uint8_t ssi_id;

    uint8_t flash_type = p_data[0];
    const app_io_type_t gpio_type[]   = {APP_IO_TYPE_GPIOA, APP_IO_TYPE_GPIOB, APP_IO_TYPE_GPIOC, APP_IO_TYPE_AON, APP_IO_TYPE_MSIO, APP_IO_TYPE_NORMAL};
    const uint32_t      gpio_pin[]    = {APP_IO_PIN_0,APP_IO_PIN_1,APP_IO_PIN_2,APP_IO_PIN_3,APP_IO_PIN_4,APP_IO_PIN_5,APP_IO_PIN_6,APP_IO_PIN_7,\
                                         APP_IO_PIN_8,APP_IO_PIN_9,APP_IO_PIN_10,APP_IO_PIN_11,APP_IO_PIN_12,APP_IO_PIN_13,APP_IO_PIN_14,APP_IO_PIN_15};
    const app_io_mux_t gpio_pin_mux[] = {APP_IO_MUX_0,APP_IO_MUX_1,APP_IO_MUX_2,APP_IO_MUX_3,APP_IO_MUX_4,APP_IO_MUX_5,APP_IO_MUX_6,APP_IO_MUX_7,APP_IO_MUX_8};

    if(flash_type ==  0x01)
    {
        //SPI flash
        flash_io->spi_cs.gpio            = gpio_type[p_data[1]];
        flash_io->spi_cs.pin             = gpio_pin[p_data[2]];
        flash_io->spi_cs.mux             = gpio_pin_mux[p_data[3]];
        flash_io->spi_clk.gpio           = gpio_type[p_data[4]];
        flash_io->spi_clk.pin            = gpio_pin[p_data[5]];
        flash_io->spi_clk.mux            = gpio_pin_mux[p_data[6]];
        flash_io->spi_io0.spim_mosi.gpio = gpio_type[p_data[7]];
        flash_io->spi_io0.spim_mosi.pin  = gpio_pin[p_data[8]];
        flash_io->spi_io0.spim_mosi.mux  = gpio_pin_mux[p_data[9]];
        flash_io->spi_io1.spim_miso.gpio = gpio_type[p_data[10]];
        flash_io->spi_io1.spim_miso.pin  = gpio_pin[p_data[11]];
        flash_io->spi_io1.spim_miso.mux  = gpio_pin_mux[p_data[12]];

        flash_init.spi_type = FLASH_SPIM_ID;
        flash_init.is_dual_line = false;
        flash_init.is_high_freq = false;

        spi_flash_init(&flash_init);
    }
    else if(flash_type ==  0x02)
    {
        //QSPI flash
        flash_io->spi_cs.gpio           = gpio_type[p_data[1]];
        flash_io->spi_cs.pin            = gpio_pin[p_data[2]];
        flash_io->spi_cs.mux            = gpio_pin_mux[p_data[3]];
        flash_io->spi_clk.gpio          = gpio_type[p_data[4]];
        flash_io->spi_clk.pin           = gpio_pin[p_data[5]];
        flash_io->spi_clk.mux           = gpio_pin_mux[p_data[6]];
        flash_io->spi_io0.qspi_io0.gpio = gpio_type[p_data[7]];
        flash_io->spi_io0.qspi_io0.pin  = gpio_pin[p_data[8]];
        flash_io->spi_io0.qspi_io0.mux  = gpio_pin_mux[p_data[9]];
        flash_io->spi_io1.qspi_io1.gpio = gpio_type[p_data[10]];
        flash_io->spi_io1.qspi_io1.pin  = gpio_pin[p_data[11]];
        flash_io->spi_io1.qspi_io1.mux  = gpio_pin_mux[p_data[12]];
        flash_io->qspi_io2.gpio         = gpio_type[p_data[13]];
        flash_io->qspi_io2.pin          = gpio_pin[p_data[14]];
        flash_io->qspi_io2.mux          = gpio_pin_mux[p_data[15]];
        flash_io->qspi_io3.gpio         = gpio_type[p_data[16]];
        flash_io->qspi_io3.pin          = gpio_pin[p_data[17]];
        flash_io->qspi_io3.mux          = gpio_pin_mux[p_data[18]];

        ssi_id = p_data[19] & 0x0F;

        flash_init.is_dual_line = false;
        flash_init.is_high_freq = false;
        flash_init.spi_type = (ssi_id == 0) ? FLASH_QSPI_ID0 : ((ssi_id == 1) ? FLASH_QSPI_ID1 : FLASH_QSPI_ID2);

        spi_flash_init(&flash_init);
    }
    else
    {
        //Unkown flash type
    }
#endif

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    uint8_t flash_type = p_data[0];
    const app_io_type_t gpio_type[]   = {APP_IO_TYPE_GPIOA, APP_IO_TYPE_GPIOB, APP_IO_TYPE_GPIOC, APP_IO_TYPE_AON, APP_IO_TYPE_MSIO, APP_IO_TYPE_NORMAL};
    const uint32_t      gpio_pin[]    = {APP_IO_PIN_0,APP_IO_PIN_1,APP_IO_PIN_2,APP_IO_PIN_3,APP_IO_PIN_4,APP_IO_PIN_5,APP_IO_PIN_6,APP_IO_PIN_7,\
                                         APP_IO_PIN_8,APP_IO_PIN_9,APP_IO_PIN_10,APP_IO_PIN_11,APP_IO_PIN_12,APP_IO_PIN_13,APP_IO_PIN_14,APP_IO_PIN_15};

    if(flash_type ==  0x01)
    {
        //SPI flash
        flash_io->spi_cs.gpio            = gpio_type[p_data[1]];
        flash_io->spi_cs.pin             = gpio_pin[p_data[2]];
        flash_io->spi_cs.mux             = (app_io_mux_t)p_data[3];
        flash_io->spi_clk.gpio           = gpio_type[p_data[4]];
        flash_io->spi_clk.pin            = gpio_pin[p_data[5]];
        flash_io->spi_clk.mux            = (app_io_mux_t)p_data[6];
        flash_io->spi_io0.spim_mosi.gpio = gpio_type[p_data[7]];
        flash_io->spi_io0.spim_mosi.pin  = gpio_pin[p_data[8]];
        flash_io->spi_io0.spim_mosi.mux  = (app_io_mux_t)p_data[9];
        flash_io->spi_io1.spim_miso.gpio = gpio_type[p_data[10]];
        flash_io->spi_io1.spim_miso.pin  = gpio_pin[p_data[11]];
        flash_io->spi_io1.spim_miso.mux  = (app_io_mux_t)p_data[12];

        flash_init.spi_type = FLASH_SPIM_ID;
        flash_init.is_dual_line = false;
        flash_init.is_high_freq = false;

        spi_flash_init(&flash_init);
    }
    else
    {
    }
#endif
}

static uint32_t dfu_spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    return spi_flash_read(address, buffer, nbytes);
}

static uint32_t dfu_spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    return spi_flash_write(address, buffer, nbytes);
}

#if defined(SOC_GR533X) || defined(SOC_GR5405)
static bool dfu_spi_flash_erase(uint32_t erase_type, uint32_t addr, uint32_t size)
{
    return spi_flash_erase(erase_type, addr, size);
}
#else
static bool dfu_spi_flash_sector_erase(uint32_t address, uint32_t size)
{
    return spi_flash_sector_erase(address, size);
}
#endif

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
static bool dfu_spi_flash_chip_erase(void)
{
    return spi_flash_chip_erase();
}
#endif

static void dfu_spi_flash_device_info(uint32_t *id, uint32_t *size)
{
    spi_flash_device_info(id, size);
}
#endif

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
static uint32_t __l_irq_rest ;
#endif


static void security_disable(void)
{
#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        s_flash_security_status = hal_flash_get_security();
        __l_irq_rest = __get_PRIMASK(); 
        __set_PRIMASK(1);
        
        hal_flash_set_security(false);
    }
#endif
}

static void security_state_recovery(void)
{
#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        hal_flash_set_security(s_flash_security_status);
        __set_PRIMASK(__l_irq_rest);  
    }
#endif
}

static uint32_t hal_flash_read_judge_security(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    uint32_t read_bytes = 0;

    security_disable();
    read_bytes = hal_flash_read(addr, buf, size);
    security_state_recovery();

    return read_bytes;
}

static void fast_dfu_write_data_to_buffer(uint8_t const *p_data, uint16_t length)
{
    ring_buffer_write(&s_ble_rx_ring_buffer, p_data, length);

    if(ring_buffer_surplus_space_get(&s_ble_rx_ring_buffer) < (ONCE_WRITE_DATA_LEN))
    {
        if(!s_ring_buffer_over_flag)
        {
            extern uint8_t g_lld_con_heap_used_ratio_limit;
            g_lld_con_heap_used_ratio_limit = 0;
            s_ring_buffer_over_flag = true;
        }
    }
}

void fast_dfu_state_machine_reset(void)
{
    s_fast_dfu_mode = 0;
    s_program_end_flag = 0;
    s_fast_dfu_state = FAST_DFU_INIT_STATE;
}


static void otas_evt_process(otas_evt_t *p_evt)
{
#ifdef ENABLE_DFU_CUSTOM_BUFFER
    if (NULL == s_p_cmd_buffer)
    {
        // disable DFU
        return;
    }
#endif
    if (s_ota_conn_index != BLE_GAP_INVALID_CONN_INDEX && s_ota_conn_index != p_evt->conn_idx)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case OTAS_EVT_TX_NOTIFICATION_ENABLED:
            dfu_cmd_parse_state_reset();
            fast_dfu_state_machine_reset();
            break;

        case OTAS_EVT_RX_RECEIVE_DATA:
            if (!s_fast_dfu_mode || s_program_end_flag)
            {
                dfu_ble_receive_data_process(p_evt->p_data, p_evt->length);
            }
            else
            {
                s_fast_dfu_state = FAST_DFU_PROGRAM_FLASH_STATE;
                fast_dfu_write_data_to_buffer(p_evt->p_data, p_evt->length);
            }
            break;

        case OTAS_EVT_NOTIFY_COMPLETE:
            dfu_ble_send_data_cmpl_process();
            break;

        case OTAS_EVT_DISCONNECT:
            if (s_ota_conn_index == p_evt->conn_idx)
            {
                s_ota_conn_index = BLE_GAP_INVALID_CONN_INDEX;
            }
            break;

        case OTAS_EVT_DFU_TASK_ENTER:
            if (s_ota_conn_index == BLE_GAP_INVALID_CONN_INDEX)
            {
                s_ota_conn_index = p_evt->conn_idx;
            }
            if(s_dfu_enter_func != NULL)
            {
                s_dfu_enter_func();
            }
            break;

        default:
            break;
    }
}


static void ble_send_data(uint8_t *p_data, uint16_t length)
{
    otas_notify_tx_data(s_ota_conn_index, p_data, length);
}

static bool wait_for_disconnection(void)
{
    // wait mobile phone to release connection.
    for (int i = 0; i < 200; i++)
    {
        uint16_t hdl = get_conn_hdl_by_idx(s_ota_conn_index);
        if (0xFFFF == hdl)
        {
            delay_ms(100);
            return true;
        }
        delay_ms(10);
    }
    // disconnect by self
    ble_gap_disconnect_with_reason(s_ota_conn_index, BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF);
    delay_ms(100);
    return false;
}

static bool check_system_info_address(uint32_t addr)
{
    return (addr >= FLASH_START_ADDR && addr < (FLASH_START_ADDR + 0x2000));
}

static void get_rom_version(uint8_t *p_read_buf)
{
    uint8_t i = 0;
    uint32_t address = SYS_ROM_VERSION_ADDR;
    while (i < 8)
    {
        *(p_read_buf + i) = *(__IO uint8_t*) address++;
        i++;
    }
}

static void get_info_replace(dfu_receive_frame_t *p_frame)
{
    sdk_version_t sdk_version;
    p_frame->data[0] = DFU_ACK_SUCCESS;
    get_rom_version(&p_frame->data[1]);

    sys_sdk_verison_get(&sdk_version);
    memcpy(&p_frame->data[9], &sdk_version, sizeof(sdk_version_t));

    p_frame->data[17] = OTAS_VERSION;

    dfu_send_frame(p_frame->data, 20, p_frame->cmd_type);
    cmd_receive_flag = 0;
}

static void dfu_mode_set(dfu_receive_frame_t *p_frame)
{
    if (p_frame->data[0] == OTAS_DFU_MODE_COPY_UPGRADE)
    {
        s_dfu_info.dfu_mode_pattern = DFU_COPY_UPGRADE_MODE_PATTERN;
        cmd_receive_flag = 0;
    }
    else
    {
        wait_for_disconnection();

        s_dfu_info.dfu_mode_pattern = DFU_NON_COPY_UPGRADE_MODE_PATTERN;

        security_disable();
        hal_flash_erase(DFU_INFO_START_ADDR, DFU_FLASH_SECTOR_SIZE);
        hal_flash_write(DFU_INFO_START_ADDR, (uint8_t*)&s_dfu_info, sizeof(s_dfu_info));
        security_state_recovery();

        hal_nvic_system_reset();
    }
}

static void dfu_fw_info_get(dfu_receive_frame_t *p_frame)
{
    extern struct otas_env_t s_otas_env;

    p_frame->data[0] = DFU_ACK_SUCCESS;
    p_frame->data[1] = s_dfu_info.dfu_fw_save_addr & 0xff;
    p_frame->data[2] = (s_dfu_info.dfu_fw_save_addr >> 8) & 0xff;
    p_frame->data[3] = (s_dfu_info.dfu_fw_save_addr >> 16) & 0xff;
    p_frame->data[4] = (s_dfu_info.dfu_fw_save_addr >> 24) & 0xff;

#ifdef BOOTLOADER_ENABLE
    p_frame->data[5] = 0x00; // app bootloader
#else
    p_frame->data[5] = 0x01; // app fw
#endif

    hal_flash_read_judge_security(APP_INFO_START_ADDR, (uint8_t *)&p_frame->data[6], sizeof(dfu_image_info_t));

    dfu_send_frame(p_frame->data, 54, p_frame->cmd_type);
    cmd_receive_flag = 0;
}

static void fast_dfu_erase_flash(void)
{
    bool report_state = false;
    bool erase_not_complete = s_erase_count < s_erase_all_count;
    // erase flash when ble idle
    if (erase_not_complete)
    {
        bool has_error = false;
        for (int i = 0; i < 20; i++)
        {
            uint32_t address = page_start_addr + (s_erase_count * DFU_FLASH_SECTOR_SIZE);
#if defined(SOC_GR533X) || defined(SOC_GR5405)
            if (!dfu_flash_erase(address, DFU_FLASH_SECTOR_SIZE))
#else
            if (dfu_flash_erase(address, DFU_FLASH_SECTOR_SIZE))
#endif
            {
                s_erase_count++;
                if (s_erase_count >= s_erase_all_count)
                {
                    erase_not_complete = false;
                    break;
                }
            }
            else
            {
                has_error = true;
                break;
            }
        }
        if (has_error)
        {
            report_state = true;
            fast_dfu_state_machine_reset(); // abort progress if error encountered
            s_p_cmd_buffer[0] = DFU_ACK_ERROR;
            s_p_cmd_buffer[1] = DFU_ERASE_FLASH_OPER_FAILED;
        }
        else
        {
            report_state = true;
            s_p_cmd_buffer[0] = DFU_ACK_SUCCESS;
            s_p_cmd_buffer[1] = DFU_ERASING_FLASH;
        }
        s_p_cmd_buffer[2] = s_erase_count & 0xff;
        s_p_cmd_buffer[3] = (s_erase_count >> 8) & 0xff;
    }

    if (!erase_not_complete)
    {
        report_state = true;
        s_fast_dfu_state = FAST_DFU_INIT_STATE;
        s_p_cmd_buffer[0] = DFU_ACK_SUCCESS;
        s_p_cmd_buffer[1] = DFU_ERASE_FLASH_END;
    }

    if (report_state)
    {
        dfu_send_frame(&s_p_cmd_buffer[0], 4, 0x0023);
        delay_ms(30);
    }
}

static void fast_dfu_cal_check_sum(uint32_t address, uint16_t len)
{
    security_disable();
    dfu_flash_read(address, s_p_fast_cache_buffer, len);
    security_state_recovery();

    for(uint16_t i=0; i<len; i++)
    {
        all_check_sum += s_p_fast_cache_buffer[i];
    }
}

static void program_start_replace(dfu_receive_frame_t *p_frame)
{
    uint8_t dfu_type = p_frame->data[0] & 0x0F;

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
    uint8_t firmware_type = (p_frame->data[0] & 0xF0) >> 4;
#else
    uint8_t firmware_type = (p_frame->data[0] & 0xF0) >> 4;
    p_all_check_sum = (uint32_t *)(dfu_env_all + ENV_CHECK_SUM_OFFSET);
    p_page_start_addr = (uint32_t *)(dfu_env_all + ENV_PAGE_START_ADDR_OFFSET);
#endif

    bool    erase_state = false;

    s_fast_dfu_mode = 0x00;

    if (dfu_type == DFU_FLASH_INNER && p_frame->data_len == (sizeof(dfu_image_info_t) + 1)) // code in flash 
    {
        memcpy(&s_now_img_info, &p_frame->data[1], sizeof(dfu_image_info_t));

        if ((s_now_img_info.pattern != DFU_IMG_INFO_PATTERN) ||\
            (s_now_img_info.boot_info.load_addr % DFU_FLASH_SECTOR_SIZE != 0) ||\
            check_system_info_address(s_now_img_info.boot_info.load_addr))
        {
            p_frame->data[0] = DFU_ACK_ERROR;
            dfu_send_frame(p_frame->data, 1, p_frame->cmd_type);
            return;
        }

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
        if (dfu_security_check_enable() == true)//security mode
        {
            s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
        }
        else
        {
            s_file_size = s_now_img_info.boot_info.bin_size + 48;
        }
        
        page_start_addr = (s_now_img_info.boot_info.load_addr & 0xfffff000);
#else
        if (firmware_type == SIGN_FIRMWARE)
        {
            s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
        }
        else
        {
            s_file_size = s_now_img_info.boot_info.bin_size + 48;
        }
        (*p_page_start_addr) = (s_now_img_info.boot_info.load_addr & 0xfffff000);
#endif
    }
    else if (dfu_type == FAST_DFU_INNER || dfu_type == FAST_DFU_SPI)
    {
        s_fast_dfu_mode = 0x01;

        if (p_frame->data_len == (sizeof(dfu_image_info_t) + 1)) // code in flash(fast)
        {
            memcpy(&s_now_img_info, &p_frame->data[1], sizeof(dfu_image_info_t));
            page_start_addr = (s_now_img_info.boot_info.load_addr & 0xfffff000);

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
            if (dfu_security_check_enable() == true)//security mode
            {
                s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
            }
            else
            {
                if (firmware_type == SIGN_FIRMWARE)
                {
                    s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
                }
                else if (firmware_type == SECURITY_SIGN_FIRMWARE)
                {
                    s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
                }
                else
                {
                    s_file_size = s_now_img_info.boot_info.bin_size + 48;
                }
            }
#else
            if (firmware_type == SIGN_FIRMWARE)
            {
                s_file_size = s_now_img_info.boot_info.bin_size + 48 + 856;
            }
            else
            {
                s_file_size = s_now_img_info.boot_info.bin_size + 48;
            }
#endif
        }
        else  // data or code in exflash or data in flash(fast)
        {
            page_start_addr = ((p_frame->data[4] << 24) | (p_frame->data[3] << 16) | (p_frame->data[2] << 8) | (p_frame->data[1]));
            s_file_size = ((p_frame->data[8] << 24) | (p_frame->data[7] << 16) | (p_frame->data[6] << 8) | (p_frame->data[5]));
            s_now_img_info.boot_info.load_addr = page_start_addr;
        }

        if (s_file_size % DFU_FLASH_SECTOR_SIZE)
        {
            s_erase_all_count = (s_file_size / DFU_FLASH_SECTOR_SIZE) + 1;
        }
        else
        {
            s_erase_all_count = s_file_size / DFU_FLASH_SECTOR_SIZE;
        }

        if (dfu_type == FAST_DFU_INNER)
        {
            dfu_flash_type_set(DFU_FLASH_INNER);
        }
        else
        {
            dfu_flash_type_set(DFU_FLASH_SPI);
        }

        s_program_address = page_start_addr;
        p_frame->data[0] = DFU_ACK_SUCCESS;
        p_frame->data[1] = DFU_ERASE_FLASH_START;
        p_frame->data[2] = s_erase_all_count & 0xff;
        p_frame->data[3] = (s_erase_all_count >> 8) & 0xff;
        dfu_send_frame(p_frame->data, 4, p_frame->cmd_type);
        s_fast_dfu_state = FAST_DFU_ERASE_FLASH_STATE;
        s_ring_buffer_over_flag = false;
        s_all_write_size = 0;
        s_erase_count = 0;
        all_check_sum = 0;
        s_program_end_flag = false;
        ring_buffer_init(&s_ble_rx_ring_buffer, s_p_ring_buffer, s_ring_buffer_size);
        ring_buffer_clean(&s_ble_rx_ring_buffer);
        dfu_program_start(s_file_size);
        cmd_receive_flag = 0;
        return ;
    }
    else// data in flash, data in exflash, code in exflash
    {
#if defined(SOC_GR533X) || defined(SOC_GR5405)
        (*p_page_start_addr) = ((p_frame->data[4] << 24) | (p_frame->data[3] << 16) | (p_frame->data[2] << 8) | (p_frame->data[1]));
        s_now_img_info.boot_info.load_addr = (*p_page_start_addr);
#else
        page_start_addr = ((p_frame->data[4] << 24) | (p_frame->data[3] << 16) | (p_frame->data[2] << 8) | (p_frame->data[1]));
        s_now_img_info.boot_info.load_addr = page_start_addr;
#endif
        s_file_size = ((p_frame->data[8] << 24) | (p_frame->data[7] << 16) | (p_frame->data[6] << 8) | (p_frame->data[5]));
    }

    if (dfu_type == DFU_FLASH_INNER)
    {
        dfu_flash_type_set(DFU_FLASH_INNER);
    }
    else
    {
        dfu_flash_type_set(DFU_FLASH_SPI);
    }

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    erase_state = dfu_flash_erase((*p_page_start_addr), DFU_FLASH_SECTOR_SIZE);
    if (!erase_state)
#else
    erase_state = dfu_flash_erase(page_start_addr, DFU_FLASH_SECTOR_SIZE);
    if (erase_state)
#endif
    {
        p_frame->data[0] = DFU_ACK_SUCCESS;
    }
    else
    {
        p_frame->data[0] = DFU_ACK_ERROR;
    }

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    *p_all_check_sum = 0;
#else
    all_check_sum = 0;
#endif

    dfu_send_frame(p_frame->data, 1, p_frame->cmd_type);
    cmd_receive_flag = 0;
    dfu_program_start(s_file_size);
    dfu_flash_type_set(DFU_FLASH_INNER);
}


#ifdef SOC_GR5526
static void program_flash_replace(dfu_receive_frame_t *p_frame)
{
    bool flash_security_status = false;
    uint8_t program_type = p_frame->data[0] & 0x0f;
    uint8_t flash_type = (p_frame->data[0] & 0xf0)>>4;
    uint32_t addr = ((p_frame->data[4] << 24) | (p_frame->data[3] << 16) | (p_frame->data[2] << 8) | (p_frame->data[1]));
    uint16_t len = ((p_frame->data[6] << 8) | (p_frame->data[5]));
    uint16_t program_state = 0;
    uint32_t write_len = 0;
    uint32_t write_addr = 0;
    uint32_t write_offset = 0;
    uint16_t temp_len = len;
    
    dfu_flash_type_set(flash_type);
    if((flash_type == DFU_FLASH_INNER) && check_system_info_address(addr))
    {
        send_frame_state(p_frame, DFU_ACK_ERROR);
        return; 
    }

    if(dfu_security_check_enable() == true)//security mode
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(false); //need Disable flash write Security auto
    }
    
    if(program_type == 0x01)// 4k 
    {
        write_addr   = addr;
        write_offset = 0;

        while(len)
        {
            write_len = len > DFU_FLASH_SECTOR_SIZE ? DFU_FLASH_SECTOR_SIZE : len;

            if((write_addr + write_len) >= (page_start_addr + DFU_FLASH_SECTOR_SIZE))
            {
                dfu_flash_erase((page_start_addr + DFU_FLASH_SECTOR_SIZE),DFU_FLASH_SECTOR_SIZE);
                page_start_addr += DFU_FLASH_SECTOR_SIZE;
            }
            program_state =  dfu_flash_write(write_addr,&(p_frame->data[7+write_offset]),write_len);
            if(program_state != 0)
            {
                all_check_sum += dfu_flash_cal_check_sum(write_addr, write_len);
            }
            else
            {
                break;
            }

            write_addr   += write_len;
            write_offset += write_len;
            len -= write_len;
        }
    }
    else if(program_type == 0x00)
    {
        program_state = dfu_flash_programe(addr,&(p_frame->data[7]),len); 
    }
    else if(program_type == 0x02)
    {
        program_state =  dfu_flash_write(addr,&(p_frame->data[7]),len);
    }
    
    if( program_state != 0)
    {
        p_frame->data[0] = DFU_ACK_SUCCESS;
        if(program_type == 0x01)
        {
           dfu_programing(temp_len); 
        }
    }
    else
    {
        p_frame->data[0] = DFU_ACK_ERROR;
    }
    
    dfu_send_frame(p_frame->data,1,p_frame->cmd_type);
    cmd_receive_flag = 0;
    
    if(dfu_security_check_enable() == true)//security mode
    {
        hal_flash_set_security(flash_security_status); //recover
    }
    dfu_flash_type_set(DFU_FLASH_INNER);
}
#endif

static void program_end_replace(dfu_receive_frame_t *p_frame)
{
    uint8_t check_result = false;
    bool reset_device_flag = false;
    uint32_t bin_check_sum = 0;
    uint8_t end_flag = p_frame->data[0] & 0x0f;

    bin_check_sum = ((p_frame->data[4] << 24) | (p_frame->data[3] << 16) | (p_frame->data[2] << 8) | (p_frame->data[1]));

    if (bin_check_sum == all_check_sum)
    {
        check_result = true;
    }

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    if (!s_fast_dfu_mode && bin_check_sum == (*p_all_check_sum))
    {
        check_result = true;
    }
#endif

    if (check_result)
    {
        p_frame->data[0] = 0x01;
    }
    else
    {
        p_frame->data[0] = 0x02;
    }

    p_frame->data[1] = end_flag;

    if (s_fast_dfu_mode)
    {
        p_frame->data[1] = all_check_sum & 0xff;
        p_frame->data[2] = (all_check_sum >> 8) & 0xff;
        p_frame->data[3] = (all_check_sum >> 16) & 0xff;
        p_frame->data[4] = (all_check_sum >> 24) & 0xff;

        dfu_send_frame(p_frame->data, 5, p_frame->cmd_type);
        s_fast_dfu_mode = 0x00;
    }
    else
    {
        dfu_send_frame(p_frame->data,1,p_frame->cmd_type);
    }

    if (check_result == 0x01)
    {
        if (end_flag == 0x01 || end_flag == 0x03)
        {
            ble_gatts_service_changed();
            dfu_program_end(check_result);

            security_disable();

            s_dfu_info.dfu_fw_save_addr =  s_now_img_info.boot_info.load_addr;

            uint32_t fw_img_info_addr = s_dfu_info.dfu_fw_save_addr + s_now_img_info.boot_info.bin_size;
#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)
            dfu_portable_flash_read(fw_img_info_addr, (uint8_t *)&s_dfu_info.dfu_img_info, sizeof(dfu_image_info_t));
#else
            hal_flash_read(fw_img_info_addr, (uint8_t *)&s_dfu_info.dfu_img_info, sizeof(dfu_image_info_t));
#endif
            hal_flash_erase(DFU_INFO_START_ADDR, DFU_FLASH_SECTOR_SIZE);

#ifdef BOOTLOADER_ENABLE
            hal_flash_erase(APP_INFO_START_ADDR, DFU_FLASH_SECTOR_SIZE);
            hal_flash_write(APP_INFO_START_ADDR, (uint8_t*)&s_dfu_info.dfu_img_info, sizeof(s_dfu_info.dfu_img_info));
#else
            hal_flash_write(DFU_INFO_START_ADDR, (uint8_t*)&s_dfu_info, sizeof(s_dfu_info));
#endif

            security_state_recovery();
            if (end_flag == 0x01)
            {
                reset_device_flag = true;
                wait_for_disconnection();
                hal_nvic_system_reset();
            }
        }
    }

    if (reset_device_flag == false)
    {
        dfu_program_end(check_result);
    }
    dfu_flash_type_set(DFU_FLASH_INNER);
    cmd_receive_flag = 0;
}

#ifdef ENABLE_DFU_CUSTOM_BUFFER
uint16_t dfu_port_init(dfu_uart_send_data uart_send_data, uint32_t dfu_fw_save_addr, dfu_pro_callback_t *p_dfu_callback, uint8_t * p_buffer, uint32_t buffer_size)
{
    if (buffer_size < (DFU_BUFFER_SIZE + FLASH_CACHE_BUFFER_SIZE)) // at least 6KB.
    {
        // disable DFU.
        s_p_cmd_buffer         = NULL;
        return SDK_ERR_INVALID_PARAM;
    }

    if (p_buffer == NULL)
    {
        // just disable DFU.
        s_p_cmd_buffer         = NULL;
        #if defined(SOC_GR5526) || defined(SOC_GR5X25) || defined(SOC_GR533X)
        s_p_flash_cache_buffer = NULL;
        #endif
        s_p_ring_buffer        = NULL;
        s_p_fast_cache_buffer  = NULL;
        return SDK_SUCCESS;
    }

    s_p_cmd_buffer         = p_buffer;
    #if defined(SOC_GR5526) || defined(SOC_GR5X25) || defined(SOC_GR533X)
    s_p_flash_cache_buffer = p_buffer + DFU_BUFFER_SIZE;
    #endif
    s_p_ring_buffer        = p_buffer;
    s_ring_buffer_size     = (uint16_t)(buffer_size - ONCE_WRITE_DATA_LEN);
    s_p_fast_cache_buffer  = p_buffer + buffer_size - ONCE_WRITE_DATA_LEN;

    #if defined(SOC_GR533X) || defined(SOC_GR5405)
    dfu_buffer.frame_buf  = s_p_cmd_buffer;
    dfu_buffer.frame_size = DFU_BUFFER_SIZE;
    dfu_buffer.jlink_buf  = NULL;
    dfu_buffer.jlink_size = 0;
    dfu_buffer.cache_buf  = s_p_flash_cache_buffer;
    dfu_buffer.cache_size = FLASH_CACHE_BUFFER_SIZE;
    dfu_buffer.env_buf    = env_data_buffer;
    dfu_buffer.env_size = ENV_BUFFER_SIZE;
    #endif

    #if defined(SOC_GR5526) || defined(SOC_GR5X25)
    dfu_buffer.rec_data_buffer      = s_p_cmd_buffer;
    dfu_buffer.rec_data_buffer_size = DFU_BUFFER_SIZE;
    dfu_buffer.flash_op_buffer      = s_p_flash_cache_buffer;
    dfu_buffer.flash_op_buffer_size = FLASH_CACHE_BUFFER_SIZE;
    #endif

#else
uint16_t dfu_port_init(dfu_uart_send_data uart_send_data, uint32_t dfu_fw_save_addr, dfu_pro_callback_t *p_dfu_callback)
{
#endif

    memset(&s_dfu_info, 0, sizeof(s_dfu_info));

    if (uart_send_data != NULL)
    {
        s_dfu_func.dfu_uart_send_data = uart_send_data;
    }

#ifdef SOC_GR5515
    dfu_init(&s_dfu_func, s_p_cmd_buffer, p_dfu_callback);
    s_dfu_info.dfu_fw_save_addr = dfu_fw_save_addr;

    dfu_set_cmd_handler(0x00, 0x01, get_info_replace);
    dfu_set_cmd_handler(0x06, 0X23, program_start_replace);
    dfu_set_cmd_handler(0x08, 0x25, program_end_replace);

    dfu_set_cmd_handler(0x1C, 0X41, dfu_mode_set);
    dfu_set_cmd_handler(0x1D, 0X42, dfu_fw_info_get);
#endif

#ifdef SOC_GR5X25
    dfu_init(&s_dfu_func, &dfu_buffer, p_dfu_callback);
    s_dfu_info.dfu_fw_save_addr = dfu_fw_save_addr;

    dfu_set_cmd_handler(0x00, 0x01, get_info_replace);
    dfu_set_cmd_handler(0x08, 0X23, program_start_replace);
    dfu_set_cmd_handler(0x0A, 0x25, program_end_replace);

    dfu_set_cmd_handler(0x1C, 0X41, dfu_mode_set);
    dfu_set_cmd_handler(0x1D, 0X42, dfu_fw_info_get);
#endif

#ifdef SOC_GR5526
    dfu_init(&s_dfu_func, &dfu_buffer, p_dfu_callback);
    s_dfu_info.dfu_fw_save_addr = dfu_fw_save_addr;

    dfu_set_cmd_handler(0x00, 0x01, get_info_replace);
    dfu_set_cmd_handler(0x07, 0X23, program_start_replace);
    dfu_set_cmd_handler(0x08, 0X24, program_flash_replace);
    dfu_set_cmd_handler(0x09, 0x25, program_end_replace);

    dfu_set_cmd_handler(0x1C, 0X41, dfu_mode_set);
    dfu_set_cmd_handler(0x1D, 0X42, dfu_fw_info_get);
#endif

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    if (SDK_SUCCESS != dfu_init(&s_dfu_func, &dfu_buffer, p_dfu_callback))
    {
        return SDK_ERR_SDK_INTERNAL;
    }
    s_dfu_info.dfu_fw_save_addr = dfu_fw_save_addr;

    dfu_set_cmd_handler(0x00, 0x01, get_info_replace);
    dfu_set_cmd_handler(0x08, 0X23, program_start_replace);
    dfu_set_cmd_handler(0x0A, 0x25, program_end_replace);

    dfu_set_cmd_handler(0x1C, 0X41, dfu_mode_set);
    dfu_set_cmd_handler(0x1D, 0X42, dfu_fw_info_get);
#endif

#ifdef ENABLE_DFU_SPI_FLASH
    dfu_spi_flash_func_config(&s_dfu_spi_flash_func);
#endif

    return SDK_SUCCESS;
}

void dfu_service_init(dfu_enter_callback dfu_enter)
{
    otas_init_t otas_init;

    if (dfu_enter != NULL)
    {
        s_dfu_enter_func = dfu_enter;
    }
    s_ota_conn_index = BLE_GAP_INVALID_CONN_INDEX;
    otas_init.evt_handler   = otas_evt_process;
    otas_service_init(&otas_init);
}

#if defined(SOC_GR533X) || defined(SOC_GR5405)
void dfu_ble_set_mtu_size(uint16_t mtu_size)
{

}
#endif

static void fast_dfu_program_schedule(void)
{
    uint16_t read_len = 0;
    uint16_t items_size = 0;

    items_size = ring_buffer_items_count_get(&s_ble_rx_ring_buffer);
    if (items_size >= ONCE_WRITE_DATA_LEN)
    {
        read_len = ring_buffer_read(&s_ble_rx_ring_buffer, s_p_fast_cache_buffer, ONCE_WRITE_DATA_LEN);

        security_disable();
#if defined(SOC_GR533X) || defined(SOC_GR5405)
        if(HAL_OK != dfu_flash_write(s_program_address, s_p_fast_cache_buffer, read_len))
#else
        if(read_len != dfu_flash_write(s_program_address, s_p_fast_cache_buffer, read_len))
#endif
        {
            fast_dfu_state_machine_reset();
        }
        security_state_recovery();

        fast_dfu_cal_check_sum(s_program_address, read_len);
        s_all_write_size += read_len;
        s_program_address += read_len;
        dfu_programing(read_len);
    }
    else
    {
        if (s_all_write_size + items_size == s_file_size)
        {
            read_len = ring_buffer_read(&s_ble_rx_ring_buffer, s_p_fast_cache_buffer, items_size);
            if (read_len)
            {
                security_disable();
                dfu_flash_write(s_program_address, s_p_fast_cache_buffer, read_len);
                security_state_recovery();

                fast_dfu_cal_check_sum(s_program_address, read_len);
                s_all_write_size += read_len;
            }
            dfu_programing(read_len);

            s_program_end_flag = true;
            s_p_cmd_buffer[0] = DFU_ACK_SUCCESS;
            dfu_send_frame(s_p_cmd_buffer, 1, 0xFF); // write over
            s_fast_dfu_state = FAST_DFU_INIT_STATE;
        }
    }

    if (s_ring_buffer_over_flag && !s_program_end_flag)
    {
        if (ring_buffer_surplus_space_get(&s_ble_rx_ring_buffer) > ONCE_WRITE_DATA_LEN)
        {
            sys_lld_max_msg_usage_ratio_set(90);
            s_ring_buffer_over_flag = false;
        }
    }
}

void fast_dfu_schedule(void)
{
    switch (s_fast_dfu_state)
    {
        case FAST_DFU_ERASE_FLASH_STATE:
            fast_dfu_erase_flash();
            break;

        case FAST_DFU_PROGRAM_FLASH_STATE:
            fast_dfu_program_schedule();
            break;

        default:
            break;
    }
}

void dfu_schedule(void)
{
    normal_dfu_schedule();
    fast_dfu_schedule();
}

uint16_t dfu_fw_image_info_get(uint32_t dfu_fw_save_addr, uint32_t fw_image_size, bool is_sign_fw, dfu_image_info_t *p_image_info)
{
    uint32_t fw_image_info_addr;
    
    if (!p_image_info)
    {
        return SDK_ERR_POINTER_NULL;
    }
    
    if (is_sign_fw)
    {
        fw_image_info_addr = dfu_fw_save_addr + fw_image_size - 856;
        if (sizeof(dfu_image_info_t) != hal_flash_read(fw_image_info_addr, (uint8_t *)p_image_info, sizeof(dfu_image_info_t)))
        {
            return SDK_ERR_SDK_INTERNAL;
        }
    }
    else
    {
        fw_image_info_addr = dfu_fw_save_addr + fw_image_size - 48;
        if (sizeof(dfu_image_info_t) != hal_flash_read(fw_image_info_addr, (uint8_t *)p_image_info, sizeof(dfu_image_info_t)))
        {
            return SDK_ERR_SDK_INTERNAL;
        }
    }

    return SDK_SUCCESS;
}


uint16_t dfu_info_update(uint32_t dfu_info_start_addr, dfu_image_info_t *p_image_info, uint32_t dfu_fw_save_addr, uint32_t dfu_mode_pattern)
{
    dfu_info_t dfu_info;

    if (!p_image_info)
    {
        return SDK_ERR_POINTER_NULL;
    }

    dfu_info.dfu_fw_save_addr = dfu_fw_save_addr;
    memcpy(&dfu_info.dfu_img_info, p_image_info, sizeof(dfu_image_info_t));
    dfu_info.dfu_mode_pattern = dfu_mode_pattern;

    if (!hal_flash_erase(dfu_info_start_addr, DFU_FLASH_SECTOR_SIZE))
    {
        return SDK_ERR_SDK_INTERNAL;
    }

    if (sizeof(dfu_info_t) != hal_flash_write(dfu_info_start_addr, (uint8_t *)&dfu_info, sizeof(dfu_info_t)))
    {
        return SDK_ERR_SDK_INTERNAL;
    }

    return SDK_SUCCESS;
}

void dfu_mode_update(uint32_t mode_pattern)
{
    if (mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN)
    {
        s_ota_conn_index = 0;
    }
}


#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)

#include "drv_adapter_norflash.h"

static volatile bool is_internal_flash = false;
static volatile bool is_external_flash = false;

static bool check_internal_flash(const uint32_t addr, uint32_t size) {
    if(((addr >= 0x00200000) && (addr + size < 0x01200000)) ||
       ((addr >= 0x02200000) && (addr + size < 0x03200000))) {
        is_internal_flash = true;
        is_external_flash = false;
        return true;
    }

    return false;
}

static bool check_external_flash(const uint32_t addr, uint32_t size, uint32_t * base_addr) {

    bool ret = false;

    if((addr >= 0xC0000000) && (addr + size < 0xC4000000)) {
        * base_addr       = 0xC0000000;
        is_internal_flash = false;
        is_external_flash = true;
        ret               = true;
    } else if((addr >= 0xC4000000) && (addr + size < 0xC8000000)) {
        * base_addr       = 0xC4000000;
        is_internal_flash = false;
        is_external_flash = true;
        ret               = true;
    } else if((addr >= 0xC8000000) && (addr + size < 0xCC000000)) {
        * base_addr       = 0xC8000000;
        is_internal_flash = false;
        is_external_flash = true;
        ret               = true;
    }

    return ret;
}

uint32_t dfu_portable_flash_read(const uint32_t addr, uint8_t * buff, const uint32_t size) {
    bool ret ;
    uint32_t base = 0;

    if(check_internal_flash(addr, size)) {
        return hal_flash_read(addr, buff, size);
    } else if(check_external_flash(addr, size, &base)) {
        ret = drv_adapter_norflash_read(addr - base, buff, size);
        if(ret) {
            return size;
        }
    }

    return 0;
}

uint32_t dfu_portable_flash_write(const uint32_t addr, const uint8_t * buff, const uint32_t size) {
    bool ret ;
    uint32_t base = 0;

    if(check_internal_flash(addr, size)) {
        return hal_flash_write(addr, buff, size);
    } else if(check_external_flash(addr, size, &base)) {
        ret = drv_adapter_norflash_write(addr - base, (uint8_t *)buff, size);
        if(ret) {
            return size;
        }
    }

    return 0;
}

static bool portable_flash_erase(const uint32_t addr, const uint32_t size) {
    uint32_t base = 0;
    bool ret = false;

    if(check_internal_flash(addr, size)) {
        return hal_flash_erase(addr, size);
    } else if(check_external_flash(addr, size, &base)) {
        uint32_t dev_addr         = addr - base;
        uint32_t sec_aligned_addr = dev_addr & 0xFFFFF000;
        uint32_t adjust_size      = (dev_addr - sec_aligned_addr) + size;
        uint32_t era_sectors      = (adjust_size/4096) + ((adjust_size%4096 > 0) ? 1 : 0);

        for(int i = 0; i < era_sectors; i++) {
            ret |= drv_adapter_norflash_erase(sec_aligned_addr + i * 4096, ADAPTER_NORFFLASH_ERASE_SECTOR);
        }

    }

    return ret;
}

static bool portable_flash_erase_chip(void) {
    if(is_internal_flash) {
        return hal_flash_erase_chip();
    }

    if(is_external_flash) {
        return drv_adapter_norflash_erase(0x00000000, ADAPTER_NORFFLASH_ERASE_CHIP);
    }

    return false;
}

#endif

