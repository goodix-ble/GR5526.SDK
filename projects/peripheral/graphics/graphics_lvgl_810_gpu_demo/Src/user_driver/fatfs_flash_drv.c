/**
 *****************************************************************************************
 *
 * @file fatfs_flash_drv.c
 *
 * @brief Flash driver implement.
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
#include "fatfs_flash_drv.h"
#include "spi_flash.h"
#include <string.h>



/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool fatfs_flash_init(void)
{
    SPI_FLASH_init();
    return true;
}

uint16_t fatfs_flash_read(uint8_t *p_data, uint16_t sector, uint16_t count)
{
    SPI_FLASH_Read(((sector*FAT_FLASH_SECTOR_SIZE)+FAT_FLASH_START_ADDR), p_data, count*FAT_FLASH_SECTOR_SIZE);
    return count;
}

uint16_t fatfs_flash_write(const uint8_t *p_write_buf, uint16_t sector, uint16_t count)
{
    uint16_t err_state = 0;
    uint32_t off_addr;
    uint16_t sec_pos;
    uint16_t sec_off;
    uint16_t sec_remain;
    uint8_t page_buf[FLASH_OP_PAGE_SIZE];
    uint16_t i = 0;
    uint32_t address = (sector*FAT_FLASH_SECTOR_SIZE)+FAT_FLASH_START_ADDR;
    uint16_t write_len = count*FAT_FLASH_SECTOR_SIZE;

    off_addr = address -FLASH_OP_START_ADDR;
    sec_pos = off_addr / FLASH_OP_PAGE_SIZE;
    sec_off = off_addr % FLASH_OP_PAGE_SIZE;
    sec_remain= FLASH_OP_PAGE_SIZE-sec_off;
    if(write_len <= sec_remain) sec_remain = write_len;
    while (1)
    {
        SPI_FLASH_Read(sec_pos*FLASH_OP_PAGE_SIZE + FLASH_OP_START_ADDR, page_buf, FLASH_OP_PAGE_SIZE);
        for (i = sec_off; i < FLASH_OP_PAGE_SIZE; i++)
        {
            if (page_buf[i] != 0xff) break;
        }
        for (i = 0; i < sec_remain; i++)
        {
            if (page_buf[sec_off + i] != 0xff)break;
        }
        if (i < sec_remain)
        {
            SPI_FLASH_Sector_Erase(sec_pos*FLASH_OP_PAGE_SIZE+FLASH_OP_START_ADDR);
            memcpy(&page_buf[sec_off], p_write_buf, sec_remain);
            err_state = SPI_FLASH_write(sec_pos*FLASH_OP_PAGE_SIZE+FLASH_OP_START_ADDR, page_buf, FLASH_OP_PAGE_SIZE);
        }
        else
        {
            err_state = SPI_FLASH_write(address, (uint8_t*)p_write_buf, sec_remain);
        }

        if (write_len == sec_remain)break;
        else
        {
            sec_pos++;
            sec_off=0;
            p_write_buf += sec_remain;
            address += sec_remain;
            write_len -= sec_remain;
            if (write_len > (FLASH_OP_PAGE_SIZE)) sec_remain = FLASH_OP_PAGE_SIZE;
            else sec_remain = write_len;
        }
    }
    return err_state;
}

