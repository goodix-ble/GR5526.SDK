#ifndef __DSPI_SCREEN_390_H__
#define __DSPI_SCREEN_390_H__

#include "app_dspi.h"
#include "app_dspi_dma.h"

#define D_DISPLAT_4W1L      (1u)
#define D_DISPLAT_4W2L      (2u)
#define D_DISPLAT_3W1L      (3u)

void display_lcd(uint8_t *data, uint32_t dataLen, uint8_t mode);
void dspi_screen_init_basic(uint8_t mode);
void dspi_screen_deinit(void);

#endif /*__DSPI_SCREEN_390_H__*/
