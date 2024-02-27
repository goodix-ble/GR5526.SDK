#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "app_dma.h"
#include "app_graphics_ospi.h"

#pragma diag_suppress   111

#define TEST_TIMES          100

#define O_PSRAM_TEST_CLOCK_FREQ             OSPI_CLOCK_FREQ_48MHz
#define O_PSRAM_TEST_DRV_STRNGTH            OSPI_PSRAM_DRV_STR_ONE_FORTH
#define O_PSRAM_TEST_RD_LATENCY             OSPI_PSRAM_RD_LATENCY_3
#define O_PSRAM_TEST_WR_LATENCY             OSPI_PSRAM_WR_LATENCY_3
#define O_PSRAM_TEST_PHY_DELAY              1
#define O_PSRAM_TEST_PREFETCH               0

#define CONCAT_RAM_SIZE                     0
#define TEST_DMA                            DMA0
#define TEST_DMA_CHANNEL                    DMA_Channel0
#define TEST_DMA_XFER_WIDTH                 8

static void     ospi_test_mcu_pressure_test(void);
static void     ospi_test_dma_pressure_test(void);
static void     ospi_test_mcu_dma_pressure_test(void);

static uint32_t test_mcu_access(uint8_t mode, uint32_t times);
static uint32_t test_dma_access(uint32_t xfer_width, uint32_t times);
static dma_id_t test_dma_init(uint8_t xfer_width);
static void     test_app_dma_evt_handler(app_dma_evt_type_t type);

static dma_id_t s_dma_id;
static volatile bool     is_xfer_cmpt = false;
static volatile bool     is_xfer_err  = false;
static uint32_t fail_times   = 0;
__ALIGNED(16) static uint8_t  x_data[4096];
extern  unsigned char ospi_test_raw_data[16384];

__ALIGNED(16) uint8_t ospi_test_raw_data_x[16384];

/************************************************************************************
 *                      PUBLIC METHODs
 ************************************************************************************/
int ospi_psram_pressure_test(void) {

    /* pinmux MCU Core Volatge to MSIO.7 */

    *((volatile uint32_t *)0xA000A80C) = 0x0000A0DD;

    /* 1. MCU Random Access */
    ospi_test_mcu_pressure_test();

    /* 2. DMA Random Access */
    ospi_test_dma_pressure_test();

    /* mix test */
    ospi_test_mcu_dma_pressure_test();

    return 0;
}


/************************************************************************************
 *                      STATIC METHODs
 ************************************************************************************/
static void ospi_test_mcu_pressure_test(void) {

    app_graphics_ospi_params_t init;

    init.ospi_freq          = O_PSRAM_TEST_CLOCK_FREQ  ;
    init.drv_strength       = O_PSRAM_TEST_DRV_STRNGTH ;
    init.rd_lc              = O_PSRAM_TEST_RD_LATENCY  ;
    init.wr_lc              = O_PSRAM_TEST_WR_LATENCY  ;
    init.phy_delay          = O_PSRAM_TEST_PHY_DELAY   ;
    init.is_read_prefetch   = O_PSRAM_TEST_PREFETCH    ;

    app_graphics_ospi_init(&init);

    uint32_t toatl_times   = 0;
    uint32_t memcpy_failed = 0;
    uint32_t u8_failed = 0;
    uint32_t u16_failed = 0;
    uint32_t u32_failed = 0;

    for(;;) {
        u32_failed    += test_mcu_access(32, TEST_TIMES);
        u16_failed    += test_mcu_access(16, TEST_TIMES);
        u8_failed     += test_mcu_access(8,  TEST_TIMES);
        memcpy_failed += test_mcu_access(0,  TEST_TIMES);
        toatl_times   += TEST_TIMES;

        printf("+++ MCU memcpy ACCESS , Failed/Total : %d/%d \r\n",   memcpy_failed, toatl_times);
        printf("+++ MCU U8  ACCESS , Failed/Total : %d/%d \r\n",      u8_failed,     toatl_times);
        printf("+++ MCU U16 ACCESS , Failed/Total : %d/%d \r\n",      u16_failed,    toatl_times);
        printf("+++ MCU U32 ACCESS , Failed/Total : %d/%d \r\n",      u32_failed,    toatl_times);
    }

    app_graphics_ospi_deinit();

    printf("\r\n\r\n");
    return;
}

static void ospi_test_dma_pressure_test(void) {

    app_graphics_ospi_params_t init;

    init.ospi_freq          = O_PSRAM_TEST_CLOCK_FREQ  ;
    init.drv_strength       = O_PSRAM_TEST_DRV_STRNGTH ;
    init.rd_lc              = O_PSRAM_TEST_RD_LATENCY  ;
    init.wr_lc              = O_PSRAM_TEST_WR_LATENCY  ;
    init.phy_delay          = O_PSRAM_TEST_PHY_DELAY   ;
    init.is_read_prefetch   = O_PSRAM_TEST_PREFETCH    ;

    app_graphics_ospi_init(&init);

    //*((uint32_t volatile *)0xA000EE20) = *((uint32_t volatile *)0xA000EE20) | 0x7FF;

    delay_ms(1000);

    uint32_t toatl_times   = 0;
    uint32_t u8_failed = 0;
    uint32_t u16_failed = 0;
    uint32_t u32_failed = 0;

    while(1) {
        test_dma_init(8);
        u8_failed += test_dma_access(8, TEST_TIMES);

        test_dma_init(16);
        u16_failed += test_dma_access(16, TEST_TIMES);

        test_dma_init(32);
        u32_failed += test_dma_access(32, TEST_TIMES);

        toatl_times += TEST_TIMES;

        printf("+++ DMA U8  ACCESS , Failed/Total : %d/%d \r\n",      u8_failed,     toatl_times);
        printf("+++ DMA U16 ACCESS , Failed/Total : %d/%d \r\n",      u16_failed,    toatl_times);
        printf("+++ DMA U32 ACCESS , Failed/Total : %d/%d \r\n",      u32_failed,    toatl_times);
    }

    app_dma_deinit(s_dma_id);
    s_dma_id = -1;
    app_graphics_ospi_deinit();

    printf("\r\n\r\n");
    return;
}

static void ospi_test_mcu_dma_pressure_test(void) {
    app_graphics_ospi_params_t init;

    init.ospi_freq          = O_PSRAM_TEST_CLOCK_FREQ  ;
    init.drv_strength       = O_PSRAM_TEST_DRV_STRNGTH ;
    init.rd_lc              = O_PSRAM_TEST_RD_LATENCY  ;
    init.wr_lc              = O_PSRAM_TEST_WR_LATENCY  ;
    init.phy_delay          = O_PSRAM_TEST_PHY_DELAY   ;
    init.is_read_prefetch   = O_PSRAM_TEST_PREFETCH    ;

    delay_ms(1000);

    app_graphics_ospi_init(&init);

    uint32_t mcu_toatl_times   = 0;
    uint32_t mcu_memcpy_failed = 0;
    uint32_t mcu_u8_failed = 0;
    uint32_t mcu_u16_failed = 0;
    uint32_t mcu_u32_failed = 0;

    uint32_t dma_toatl_times   = 0;
    uint32_t dma_u8_failed = 0;
    uint32_t dma_u16_failed = 0;
    uint32_t dma_u32_failed = 0;

    for(;;) {
        mcu_u32_failed    += test_mcu_access(32, TEST_TIMES);
        mcu_u16_failed    += test_mcu_access(16, TEST_TIMES);
        mcu_u8_failed     += test_mcu_access(8,  TEST_TIMES);
        mcu_memcpy_failed += test_mcu_access(0,  TEST_TIMES);
        mcu_toatl_times   += TEST_TIMES;

        printf("+++ MCU memcpy ACCESS , Failed/Total : %d/%d \r\n",   mcu_memcpy_failed, mcu_toatl_times);
        printf("+++ MCU U8  ACCESS , Failed/Total : %d/%d \r\n",      mcu_u8_failed,     mcu_toatl_times);
        printf("+++ MCU U16 ACCESS , Failed/Total : %d/%d \r\n",      mcu_u16_failed,    mcu_toatl_times);
        printf("+++ MCU U32 ACCESS , Failed/Total : %d/%d \r\n",      mcu_u32_failed,    mcu_toatl_times);

        test_dma_init(8);
        dma_u8_failed += test_dma_access(8, TEST_TIMES);

        test_dma_init(16);
        dma_u16_failed += test_dma_access(16, TEST_TIMES);

        test_dma_init(32);
        dma_u32_failed += test_dma_access(32, TEST_TIMES);

        dma_toatl_times += TEST_TIMES;

        printf("+++ DMA U8  ACCESS , Failed/Total : %d/%d \r\n",      dma_u8_failed,     dma_toatl_times);
        printf("+++ DMA U16 ACCESS , Failed/Total : %d/%d \r\n",      dma_u16_failed,    dma_toatl_times);
        printf("+++ DMA U32 ACCESS , Failed/Total : %d/%d \r\n",      dma_u32_failed,    dma_toatl_times);
    }
}


uint8_t tx_data[256];
uint8_t rx_data[256];



static void memcpy_u8(uint8_t * dst, uint8_t * src, uint32_t length) {
    while(length--) {
        *dst++ = *src++;
    }
}

static void memcpy_u16(uint16_t * dst, uint16_t * src, uint32_t length) {
    uint32_t len = length/2;
    while(len--) {
        *dst++ = *src++;
    }
}

static void memcpy_u32(uint32_t * dst, uint32_t * src, uint32_t length) {
    uint32_t len = length/4;
    while(len--) {
        *dst++ = *src++;
    }
}


static uint32_t test_mcu_access(uint8_t mode, uint32_t times) {
    uint32_t base_addr = app_graphics_ospi_get_base_address();
    uint32_t i;
    uint32_t actual_addr = 0;

    for( i = 0; i< 256; i++) {
        tx_data[i] = i % 256;
    }

    fail_times = 0;

    switch(mode) {
        case 0:
        {
            for(i = 0; i < times ; i ++) {
                actual_addr = base_addr + i % 0x600000;

                memcpy((void*)actual_addr, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10);
                memcpy(&ospi_test_raw_data_x[0], (void*)actual_addr, sizeof(ospi_test_raw_data)/10);
                if(memcmp((void*)ospi_test_raw_data_x, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10) != 0){
                    fail_times ++;
                }
            }
        }
        break;

        case 8:
        {
            for(i = 0; i < times ; i ++) {
                actual_addr = base_addr + i % 0x600000;

                memcpy_u8((void*)actual_addr, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10);
                memcpy_u8(&ospi_test_raw_data_x[0], (void*)actual_addr, sizeof(ospi_test_raw_data)/10);
                if(memcmp((void*)ospi_test_raw_data_x, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10) != 0){
                    fail_times ++;
                }
            }
        }
        break;

        case 16:
        {
            for(i = 0; i < times ; i ++) {
                actual_addr = base_addr + (i*2) % 0x600000;

                memcpy_u16((void*)actual_addr, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10);
                memcpy_u16((uint16_t*)&ospi_test_raw_data_x[0], (uint16_t*)actual_addr, sizeof(ospi_test_raw_data)/10);
                if(memcmp((void*)ospi_test_raw_data_x, (void*)&ospi_test_raw_data[0], sizeof(ospi_test_raw_data)/10) != 0){
                    fail_times ++;
                }
            }
        }
        break;

        case 32:
        {
            for(i = 0; i < times ; i ++) {
                actual_addr = base_addr + (i*4) % 0x600000;

                memcpy_u32((void*)actual_addr, (void*)&ospi_test_raw_data[0], 6000);
                memcpy_u32((uint32_t*)&ospi_test_raw_data_x[0], (uint32_t*)actual_addr, 6000);
                if(memcmp((void*)ospi_test_raw_data_x, (void*)&ospi_test_raw_data[0], 6000) != 0){
                    fail_times ++;
                }
            }
        }
        break;
    }

    return fail_times;
}

void dump_data(char * data, uint32_t len) {
    for(uint32_t i = 0; i < len / 16; i++) {
        for(uint32_t j = 0; j < 16; j++) {
            printf("%02x ",data[i*16 + j]);
        }
        printf("\r\n");
    }

}

static uint32_t test_dma_access(uint32_t xfer_width, uint32_t times) {
    uint32_t base_addr = app_graphics_ospi_get_base_address();
    uint32_t i;
    const uint32_t delta = 0x100;
    uint32_t beat_length = 0;
    uint32_t actual_addr = 0, offset = 0;
    offset = 0;


    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 2000;
    } else {
        beat_length = 1000;
    }

    fail_times = 0;
    for(i = 0; i < times ; i++) {
        actual_addr = base_addr + (i * delta) % 0x600000;

        offset = (((times >> 2) << 2) % 10) * 1024;

        is_xfer_cmpt = false;
        is_xfer_err  = false;
        app_dma_start(s_dma_id, (uint32_t)&ospi_test_raw_data[offset], actual_addr, beat_length);
        while(!is_xfer_cmpt){}
        delay_ms(1);

        memset(&x_data[0], 0, 4096);
        is_xfer_cmpt = false;
        is_xfer_err  = false;
        app_dma_start(s_dma_id,  (uint32_t)actual_addr, (uint32_t)&x_data[0], beat_length);
        while(!is_xfer_cmpt){}

        is_xfer_err = is_xfer_err;
        if(memcmp((void*)&x_data[0], &ospi_test_raw_data[offset], beat_length) != 0){
            fail_times ++;
            //dump_data((char*)x_data, 64);
        }
    }

    return fail_times;
}


static dma_id_t test_dma_init(uint8_t xfer_width) {
    app_dma_params_t dma_params = {0};

    dma_params.p_instance                 = TEST_DMA;
    dma_params.channel_number             = TEST_DMA_CHANNEL;
    dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;
    dma_params.init.src_request           = (TEST_DMA == DMA0) ? DMA0_REQUEST_MEM : DMA1_REQUEST_MEM;
    dma_params.init.dst_request           = (TEST_DMA == DMA0) ? DMA0_REQUEST_MEM : DMA1_REQUEST_MEM;

    if(xfer_width == 8) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    } else if(xfer_width == 16) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    } else if(xfer_width == 32) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }

    s_dma_id = app_dma_init(&dma_params, test_app_dma_evt_handler);
    if (s_dma_id < 0)
    {
        printf("DMA Init Failed!\r\n");
    }

    return s_dma_id;
}

static void test_app_dma_evt_handler(app_dma_evt_type_t type) {
    switch(type)  {
        case APP_DMA_EVT_TFR:
        case APP_DMA_EVT_BLK:
            is_xfer_cmpt = true;
            is_xfer_err  = false;
            break;
        case APP_DMA_EVT_ERROR:
            is_xfer_cmpt = true;
            is_xfer_err  = true;
            printf("DMA Xfer Err...\r\n");
            break;
    }

    return;
}


