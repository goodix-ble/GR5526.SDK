#ifndef __GRAPHICS_DEFS__H__
#define __GRAPHICS_DEFS__H__

#include "hal_gfx_hal.h"
#include "hal_gfx_utils.h"

#include "graphics_sys_defs.h"
#include "graphics_dc_lcd_drv.h"

#define LCD_RES_CURRENT             LCD_RES_454


#define LCD_IS_FLS_AMO139           0u      /* 1 : fls am0139;
                                                    :: SUPPORT 454 & 360
                                               0 : xsj
                                                    :: SUPPORT 454 & 390
                                               */


#if (LCD_RES_CURRENT == LCD_RES_454)
    #define DISPLAY_LCD_RES_X       454u
    #define DISPLAY_LCD_RES_Y       454u
#elif (LCD_RES_CURRENT == LCD_RES_390)
    #define DISPLAY_LCD_RES_X       390u
    #define DISPLAY_LCD_RES_Y       390u
#else
    #error "Not Support !"
#endif

#endif  /* __GRAPHICS_DEFS__H__ */
