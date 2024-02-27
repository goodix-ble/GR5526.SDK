#include "flash_diskio.h"
#include <string.h>
#include "ff_gen_drv.h"

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS g_stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
#if FF_USE_DISK_IOCTL == 1
DRESULT flash_ioctl (BYTE, BYTE, void*);
#endif  /* FF_USE_DISK_IOCTL == 1 */

#if FF_USE_DISK_WRITE == 1
DRESULT flash_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* FF_USE_DISK_WRITE == 1 */

DRESULT flash_read( BYTE, BYTE*, DWORD, UINT);
DSTATUS flash_status( BYTE);
DSTATUS flash_initialize( BYTE);

diskio_drv_t g_flash_driver = { flash_initialize, flash_status, flash_read,
#if  FF_USE_DISK_WRITE == 1
        flash_write,
#endif /* FF_USE_DISK_WRITE == 1 */

#if  FF_USE_DISK_IOCTL == 1
        flash_ioctl,
#endif /* FF_USE_DISK_IOCTL == 1 */
        };

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  SD I/O control
 * @param  lun : not used
 * @param  cmd: Control code
 * @param  *buff: Buffer pointer to in/out data
 * @retval DRESULT: Operation result
 */
#if FF_USE_DISK_IOCTL == 1
DRESULT flash_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    
    if (g_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
        /* Ensure no pending write process */
        case CTRL_SYNC :
        {
            res = RES_OK;
            break;
        }

        /* Get sectors number on the disk (DWORD) */
        case GET_SECTOR_COUNT :
        {
            *(DWORD*)buff = FAT_FLASH_SECTOR_COUNT;
            res = RES_OK;
            break;
        }

        /* Get R/W sector size (WORD) */
        case GET_SECTOR_SIZE :
        {
            *(WORD*)buff = FAT_FLASH_SECTOR_SIZE;
            res = RES_OK;
            break;
        }

        /* Get erase block size in unit of sector (DWORD) */
        case GET_BLOCK_SIZE :
        {
            *(DWORD*)buff = FAT_FLASH_BLOCK_SIZE;
            break;
        }

        default:
        {
            res = RES_PARERR;
        }
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

/**
 * @brief  Writes flashf Sector(s)
 * @param  lun : not used
 * @param  *buff: Data buffer pointer.
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors(1-128)
 * @retval DRESULT: Operation result
 */
#if FF_USE_DISK_WRITE == 1
DRESULT flash_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_OK;

    if(fatfs_flash_write(buff, sector, count) == 0)
    {
        res = RES_ERROR;
    }

    return res;
}
#endif /* _USE_WRITE == 1 */

/**
 * @brief  Reads flash Sector(s)
 * @param  lun : not used
 * @param  *buff: Data buffer pointer.
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors(1-128)
 * @retval DRESULT: Operation result
 */
DRESULT flash_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_OK;

    if (fatfs_flash_read(buff, sector, count) ==  0)
    {
        res = RES_ERROR;
    }

    return res;
}

/**
 * @brief  Get flash Status
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS flash_status(BYTE lun)
{
    g_stat = !STA_NOINIT;
    return g_stat;
}

/**
 * @brief  Initializes a Drive
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS flash_initialize(BYTE lun)
{
    g_stat = STA_NOINIT;

    /* flash init*/
    if (fatfs_flash_init())
    {
        g_stat &= ~STA_NOINIT;
    }

    return g_stat;
}
