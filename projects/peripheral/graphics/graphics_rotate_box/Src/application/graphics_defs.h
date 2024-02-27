#ifndef __GRAPHICS_DEFS__H__
#define __GRAPHICS_DEFS__H__

#include "hal_gfx_hal.h"
#include "hal_gfx_utils.h"

#include "graphics_sys_defs.h"
#include "graphics_dc_lcd_drv.h"

#define LCD_RES_CURRENT             LCD_RES_454

#define _AT_RAM                     1u          /* Set FrameBuffer in RAM Space */
#define _AT_PSRAM                   2u          /* Set FrameBuffer in PSRAM Space */
#define _AT_MIXED                   3u          /* Set FrameBuffer in Concated PSRAM & RAM Space */

#define FRAMEBUFFER_LOCATED         _AT_RAM

#define HAL_GDC_DISPLAY_ENABLE       1u

/*
 * HAL_GFX_RGBA8888    - OK
 * HAL_GFX_ARGB8888    - OK
 * HAL_GFX_ABGR8888    - OK
 * HAL_GFX_BGRA8888    - OK
 * HAL_GFX_RGB565      - OK
 * HAL_GFX_RGB24       - OK // lines must equals 3xN
 * HAL_GFX_BGR24       - OK // lines must equals 3xN
 * HAL_GFX_TSC6A       - OK
 */
#define HAL_GFX_FB_FORMAT              HAL_GFX_RGBA8888
#define HAL_GFX_IMAGE_FORMAT           HAL_GFX_RGBA8888
#define HAL_GFX_DC_FORMAT              HAL_GDC_RGBA8888

#if (FRAMEBUFFER_LOCATED == _AT_RAM)
    #define SHOW_AREA_X             200u
    #define SHOW_AREA_Y             200u
#else
    #define SHOW_AREA_X             200u
    #define SHOW_AREA_Y             200u
#endif
    #define PIXEL_DEPTH_BYTES       4u
    #define PIXEL_SHOW_BYTES        3u

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

