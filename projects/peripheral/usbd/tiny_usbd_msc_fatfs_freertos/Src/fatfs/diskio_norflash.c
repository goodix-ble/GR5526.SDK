#include "ff.h"
#include "diskio.h"

#include "drv_adapter_norflash.h"
#include "qspi_norflash_v2.h"
#include "gr552xx.h"
#include "app_qspi.h"

#include "app_rtc.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static uint32_t s_norflash_sectors = 0;

DSTATUS disk_status(BYTE pdrv)
{
    (void)pdrv;
    return s_norflash_sectors ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    (void)pdrv;

    drv_adapter_norflash_set_mmap_mode(false);
    s_norflash_sectors = qspi_norf_read_dev_density() / FF_MIN_SS;
    return s_norflash_sectors ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;

    if (sector + count < s_norflash_sectors)
    {
        drv_adapter_norflash_set_mmap_mode(true);
        app_qspi_mmap_set_endian_mode(APP_QSPI_ID_0, APP_QSPI_MMAP_ENDIAN_MODE_2);
        memcpy(buff, (void *)(QSPI0_XIP_BASE + sector * FF_MIN_SS), count * FF_MIN_SS);

        return RES_OK;
    }

    return RES_PARERR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;

    if (sector + count < s_norflash_sectors)
    {
        drv_adapter_norflash_set_mmap_mode(false);
        drv_adapter_norflash_update(sector * FF_MIN_SS, (uint8_t *)buff, count * FF_MIN_SS);

        return RES_OK;
    }
    return RES_PARERR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;

    DRESULT ret = RES_OK;
    switch (cmd)
    {
    case CTRL_SYNC:
        // Do nothing since flash write is synchronized
        break;

    case GET_SECTOR_COUNT:
        *(DWORD *)buff = s_norflash_sectors;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = FF_MIN_SS;
        break;

    default:
        ret = RES_PARERR;
        break;
    }

    return ret;
}

DWORD get_fattime(void)
{
    app_rtc_time_t time;
    app_rtc_get_time(&time);

    return (DWORD)(time.year + 30) << 25 |
           (DWORD)time.mon << 21 |
           (DWORD)time.date << 16 |
           (DWORD)time.hour << 11 |
           (DWORD)time.min << 5 |
           (DWORD)time.sec >> 1;
}
