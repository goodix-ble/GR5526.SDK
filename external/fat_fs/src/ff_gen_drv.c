#include "ff_gen_drv.h"

/* Private variables ---------------------------------------------------------*/
disk_drv_t g_disk = {{0},{0},{0},0};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Gets the current linked drivers number.
  * @param  None
  * @retval Number of attached drivers.
  */
uint8_t fatfs_get_attached_drivers_nbr(void)
{
  return g_disk.nbr;
}

/**
  * @brief  Unattach a diskio driver from disk driver.
  * @param  p_path: pointer to the logical drive path
  * @retval Returns 0: success, 1: others.
  */
uint8_t fatfs_deinit_driver(char *p_path)
{
  return fatfs_deinit_driver_ex(p_path, 0);
}

/**
  * @brief  Unattach a diskio driver from disk driver.
  * @param  p_path: pointer to the logical drive path
  * @param  lun : Not used here
  * @retval Returns 0: success, 1: others.
  */
uint8_t fatfs_deinit_driver_ex(char *p_path, uint8_t lun)
{
  uint8_t disk_num = 0;
  uint8_t ret = 1;

  if(g_disk.nbr >= 1)
  {
    disk_num = p_path[0] - '0';
    if(g_disk.drv[disk_num] != 0)
    {
      g_disk.nbr--;
      g_disk.lun[disk_num] = 0;
      g_disk.drv[disk_num] = 0;
      ret = 0;
    }
  }

  return ret;
}

/**
  * @brief  Attach diskio driver to disk driver base on volumes.
  * @note   The Max number of linked drivers is 10.
  * @param  p_drv: pointer to the disk IO Driver structure
  * @param  p_path: pointer to the logical drive path
  * @retval Returns 0: success, 1: others.
  */
uint8_t fatfs_init_driver(diskio_drv_t *p_drv, char *p_path)
{
  return fatfs_init_driver_ex(p_drv, p_path, 0);
}

/**
  * @brief  Attach diskio driver to disk driver base on volumes.
  * @note   The Max number of linked drivers is 10.
  * @param  p_drv: pointer to the disk IO Driver structure
  * @param  p_path: pointer to the logical drive path
  * @param  lun : Not used here
  * @retval Returns 0: success, 1: others.
  */
uint8_t fatfs_init_driver_ex(diskio_drv_t *p_drv, char *p_path, uint8_t lun)
{
  uint8_t disk_num = 0;
  uint8_t ret = 1;

  if(g_disk.nbr <= FF_VOLUMES)
  {
    g_disk.lun[g_disk.nbr] = lun;
    g_disk.drv[g_disk.nbr] = p_drv;
    g_disk.is_initialized[g_disk.nbr] = 0;
    disk_num = g_disk.nbr++;
    p_path[0] = disk_num + '0';
    p_path[1] = ':';
    p_path[2] = '/';
    p_path[3] = 0;
    ret = 0;
  }

  return ret;
}
