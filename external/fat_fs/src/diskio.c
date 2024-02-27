#include "ff.h"
#include "diskio.h"
#include "ff_gen_drv.h"

#define DEV_RAM     0   /* Give Ramdisk to physical drive 0 */
#define DEV_MMC     1   /* Give MMC/SD card to physical drive 1 */
#define DEV_USB     2   /* Give USB MSD to physical drive 2 */
/* Private variables ---------------------------------------------------------*/
extern disk_drv_t  g_disk;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  I/O control access
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Command code
  * @param  *buff: Data buffer pointer
  * @retval DRESULT: Operation result
  */
#if FF_USE_DISK_IOCTL == 1
DRESULT disk_ioctl (
    BYTE pdrv,
    BYTE cmd,
    void *buff
)
{
  DRESULT res;

  res = g_disk.drv[pdrv]->disk_ioctl(g_disk.lun[pdrv], cmd, buff);

  return res;
}
#endif /* _USE_IOCTL == 1 */

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buff pointer
  * @param  sector: Sector address(LBA)
  * @param  count: Number of sectors(1..128)
  * @retval DRESULT: Operation result
  */
#if FF_USE_DISK_WRITE == 1
DRESULT disk_write (
    BYTE pdrv,
    const BYTE *buff,
    DWORD sector,
    UINT count
)
{
  DRESULT res;

  res = g_disk.drv[pdrv]->disk_write(g_disk.lun[pdrv], buff, sector, count);

  return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer pointer
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors(1..128)
  * @retval DRESULT: Operation result
  */
DRESULT disk_read (
    BYTE pdrv,
    BYTE *buff,
    DWORD sector,
    UINT count
)
{
  DRESULT res;

  res = g_disk.drv[pdrv]->disk_read(g_disk.lun[pdrv], buff, sector, count);

  return res;
}

/**
  * @brief  Disk initializes
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_initialize (
    BYTE pdrv
)
{
  DSTATUS stat = RES_OK;

  if(g_disk.is_initialized[pdrv] == 0)
  {
    g_disk.is_initialized[pdrv] = 1;
    stat = g_disk.drv[pdrv]->disk_initialize(g_disk.lun[pdrv]);
  }
  return stat;
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_status (
    BYTE pdrv
)
{
  DSTATUS stat;

  stat = g_disk.drv[pdrv]->disk_status(g_disk.lun[pdrv]);

  return stat;
}
