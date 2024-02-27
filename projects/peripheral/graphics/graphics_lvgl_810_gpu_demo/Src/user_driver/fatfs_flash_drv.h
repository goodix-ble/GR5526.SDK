/**
 ****************************************************************************************
 *
 * @file fatfs_flash_drv.h
 *
 * @brief Fatfs flash API
 *
 ****************************************************************************************
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
#ifndef _FATFS_FLASH_DRV_H_
#define _FATFS_FLASH_DRV_H_
#include <stdint.h>
#include <stdbool.h>


/**
 * @defgroup FATFS_FLASH_DRV_MAROC Defines
 * @{
 */
#define FLASH_OP_START_ADDR   0x00000000          /**< Flash start address.*/
#define FLASH_OP_PAGE_SIZE    0x1000              /**< Flash one page size*/

#define FAT_FLASH_START_ADDR      0x00000000      /**< Fatfs flash start address*/
#define FAT_FLASH_SIZE           (1024 * 1024)      /**< Fatfs flash size*/


#define FAT_FLASH_SECTOR_SIZE     (512)          
#define FAT_FLASH_BLOCK_SIZE      (FLASH_OP_PAGE_SIZE/FAT_FLASH_SECTOR_SIZE)

#define FAT_FLASH_SECTOR_COUNT    (FAT_FLASH_SIZE/FAT_FLASH_SECTOR_SIZE)


/**
 * @defgroup FATFS_FLASH_DRV_FUNCTION Functions
 * @{
 */
 
 /**
 *****************************************************************************************
 * @brief Initialize flash.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
bool fatfs_flash_init(void);

/**
 *****************************************************************************************
 * @brief Fatfs flash read.
 *
 * @param[in] p_data: Data buffer pointer.
 * @param[in] sector: Sector address.
 * @param[in] count:  Number of sectors(1-128)
 *
 * @return Operation result.
 *****************************************************************************************
 */
uint16_t fatfs_flash_read(uint8_t *p_data, uint16_t sector, uint16_t count);

/**
 *****************************************************************************************
 * @brief Fatfs flash write.
 *
 * @param[in] p_write_buf: Data buffer pointer.
 * @param[in] sector: Sector address.
 * @param[in] count:  Number of sectors(1-128)
 *
 * @return Operation result.
 *****************************************************************************************
 */
uint16_t fatfs_flash_write(const uint8_t *p_write_buf, uint16_t sector, uint16_t count);


#endif


