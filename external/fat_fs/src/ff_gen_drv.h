/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FF_GEN_DRV_H__
#define __FF_GEN_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "ff.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Disk IO Driver structure definition
 */
typedef struct
{
     /*!< Initialize Disk Drive */
    DSTATUS (*disk_initialize) (BYTE);
     /*!< Get Disk Status */
    DSTATUS (*disk_status) (BYTE);
     /*!< Read Sector(s) */
    DRESULT (*disk_read) (BYTE, BYTE*, DWORD, UINT);
#if FF_USE_DISK_WRITE == 1
    /*!< Write Sector(s) when _USE_WRITE = 0       */
    DRESULT (*disk_write) (BYTE, const BYTE*, DWORD, UINT);
#endif /* FF_USE_DISK_WRITE == 1 */
#if FF_USE_DISK_IOCTL == 1
    /*!< I/O control operation when _USE_IOCTL = 1 */
    DRESULT (*disk_ioctl) (BYTE, BYTE, void*);
#endif /* FF_USE_DISK_IOCTL == 1 */

}diskio_drv_t;

/**
 * @brief  Global Disk IO Drivers structure definition
 */
typedef struct
{
    uint8_t is_initialized[FF_VOLUMES];
    diskio_drv_t *drv[FF_VOLUMES];
    uint8_t lun[FF_VOLUMES];
    __IO uint8_t nbr;

}disk_drv_t;

/* Exported functions ------------------------------------------------------- */
uint8_t fatfs_get_attached_drivers_nbr(void);
uint8_t fatfs_deinit_driver_ex(char *p_path, BYTE lun);
uint8_t fatfs_init_driver_ex(diskio_drv_t *p_drv, char *p_path, BYTE lun);
uint8_t fatfs_init_driver(diskio_drv_t *p_drv, char *p_path);
uint8_t fatfs_deinit_driver(char *p_path);

#ifdef __cplusplus
}
#endif

#endif /* __FF_GEN_DRV_H__ */
