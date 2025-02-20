/**
 *****************************************************************************************
 *
 * @file dfu_port.h
 *
 * @brief  DFU port API.
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
#ifndef _DFU_PORT_H__
#define _DFU_PORT_H__

#include "gr_includes.h"
#include "otas.h"
#include "gr5xx_dfu.h"

#define DFU_COPY_UPGRADE_MODE_PATTERN       0x44425942                                                    /**< Double Bank update mode. */
#define DFU_NON_COPY_UPGRADE_MODE_PATTERN   0x53424e42                                                    /**< Single Bank update mode. */

#define DFU_FLASH_SECTOR_SIZE               4096                                                          /**< Flash sector size. */
#define DFU_INFO_START_ADDR                 (FLASH_START_ADDR + 0x3000)                                   /**< The start address of dfu info. */
#define DFU_FW_IMG_INFO_ADDR                (DFU_INFO_START_ADDR + 4)                                     /**< The start address of img info. */
#define DFU_MODE_PATTER_ADDR                (DFU_INFO_START_ADDR + 4 + sizeof(dfu_image_info_t))          /**< The address of update mode pattern. */
#define APP_INFO_START_ADDR                 (FLASH_START_ADDR + 0x2000)                                   /**< The address of app info. */
#define CHIP_REGS_BASE_ADDR_SEC             (PERIPH_BASE + 0x10000)                                       /**< The address of chip regs. */

#define DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526     0u          /* Support external flash to cache App Image when running DTU/OTA */

/**@brief DFU info. */
typedef struct
{
    uint32_t            dfu_fw_save_addr;                                                                 /**< The save addr of dfu firmware. */
    dfu_image_info_t    dfu_img_info;                                                                     /**< The img_info of dfu firmware. */
    uint32_t            dfu_mode_pattern;                                                                 /**< The dfu mode pattern. */
} dfu_info_t;

/**@brief DFU uart send data function definition. */
typedef void (*dfu_uart_send_data)(uint8_t *p_data, uint16_t length);

/**@brief DFU enter callback definition. */
typedef void (*dfu_enter_callback)(void);

/**
 *****************************************************************************************
 * @brief DFU BLE service init.
 *
 * @param[in] dfu_enter: DFU enter callback.
 *****************************************************************************************
 */
void dfu_service_init(dfu_enter_callback dfu_enter);

/**
 *****************************************************************************************
 * @brief DFU port init.
 * @details If not using serial port update function, uart_send_data can be set NULL.
            if the user doesn't care about the upgrade status,p_dfu_callback can set NULL.
 *
 * @param[in] uart_send_data  : Function is used to send data to master by UART.
 * @param[in] dfu_fw_save_addr: The start address of the upgraded firmware stored in flash
 * @param[in] p_dfu_callback  : DFU program state callback functions.
 * @param[in] p_buffer        : The buffer is allocated by user. Set it to NULL to disable DFU and then recycle buffer.
 * @param[in] buffer_size     : The size of buffer. 6KB at least.
 *****************************************************************************************
 */
#ifdef ENABLE_DFU_CUSTOM_BUFFER
uint16_t dfu_port_init(dfu_uart_send_data uart_send_data, uint32_t dfu_fw_save_addr, dfu_pro_callback_t *p_dfu_callback, uint8_t * p_buffer, uint32_t buffer_size);
#else
uint16_t dfu_port_init(dfu_uart_send_data uart_send_data, uint32_t dfu_fw_save_addr, dfu_pro_callback_t *p_dfu_callback);
#endif

/**
 *****************************************************************************************
 * @brief Function for checking DFU cmd.
 *
 * @note This function should be called in main loop.
 *****************************************************************************************
 */
void dfu_schedule(void);


/**
 *****************************************************************************************
 * @brief Function for reset fast dfu state.
 *
 * @note This function should be called in dfu end or dfu cmd start.
 *****************************************************************************************
 */
void fast_dfu_state_machine_reset(void);

/**
 ****************************************************************************************
 * @brief  get the dfu firmware image info.
 *
 * @param[in]  dfu_fw_save_addr: The dfu firmware save address
 * @param[in]  fw_image_size   : The size of firmware 
 * @param[in]  is_sign_fw      : Whether it is an sign firware
 * @param[out] p_image_info    : The temporary variables that save imag_info
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t dfu_fw_image_info_get(uint32_t dfu_fw_save_addr, uint32_t fw_image_size, bool is_sign_fw, dfu_image_info_t *p_image_info);


/**
 ****************************************************************************************
 * @brief  Update the dfu info
 *
 * @param[in]  dfu_info_start_addr : The start address of dfu info
 * @param[in]  p_image_info        : The image info of update
 * @param[in]  dfu_fw_save_addr    : The start address of dfu firmware
 * @param[in]  dfu_mode_pattern    : The dfu mode pattern
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t dfu_info_update(uint32_t dfu_info_start_addr, dfu_image_info_t *p_image_info,  uint32_t dfu_fw_save_addr, uint32_t dfu_mode_pattern);

/**
 ****************************************************************************************
 * @brief  Update the dfu mode
 *
 * @param[in]  mode_pattern : DFU_COPY_UPGRADE_MODE_PATTERN or DFU_NON_COPY_UPGRADE_MODE_PATTERN
 ****************************************************************************************
 */
void dfu_mode_update(uint32_t mode_pattern);

#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)

/**
 ****************************************************************************************
 * @brief  Read Flash in portable mode, identify the flash by address
 *
 * @param[in]  addr : The start address to read
 *             when address space belongs to [0x00200000, 0x01200000) or [0x02200000, 0x03200000), Identify as X-QSPI space
 *             when address space belongs to [0xC0000000, 0xC4000000), Identify as external QSPI0 space
 *             when address space belongs to [0xC4000000, 0xC8000000), Identify as external QSPI1 space
 *             when address space belongs to [0xC8000000, 0xCC000000), Identify as external QSPI2 space
 * @param[in]  buff : the buffer to cache read data
 * @param[in]  size : length of data to read
 *
 * @return length of read data.
 ****************************************************************************************
 */
uint32_t dfu_portable_flash_read(const uint32_t addr, uint8_t * buff, const uint32_t size);

/**
 ****************************************************************************************
 * @brief  Write Flash in portable mode, identify the flash by address
 *
 * @param[in]  addr : The start address to write
 *             when address space belongs to [0x00200000, 0x01200000) or [0x02200000, 0x03200000), Identify as X-QSPI space
 *             when address space belongs to [0xC0000000, 0xC4000000), Identify as external QSPI0 space
 *             when address space belongs to [0xC4000000, 0xC8000000), Identify as external QSPI1 space
 *             when address space belongs to [0xC8000000, 0xCC000000), Identify as external QSPI2 space
 * @param[in]  buff : the buffer to cache write data
 * @param[in]  size : length of data to write
 *
 * @return length of read data.
 ****************************************************************************************
 */
uint32_t dfu_portable_flash_write(const uint32_t addr, const uint8_t * buff, const uint32_t size);

#endif

/** @} */

#endif


