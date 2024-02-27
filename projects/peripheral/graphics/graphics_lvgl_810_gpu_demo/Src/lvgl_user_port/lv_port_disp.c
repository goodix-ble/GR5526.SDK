
/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include "lv_conf.h"
#include "bsp_lcd.h"
#include "app_io.h"
#include "graphics_dc_lcd_drv.h"
#include "graphics_defs.h"
#include "hal_gfx_hal.h"
#include "hal_gfx_utils.h"
#include "graphics_sys_defs.h"
#include "graphics_dc_lcd_drv.h"
#include "disp_driver.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"

/**********************
 *   LOCAL FUNCTIONS AND VARIABLES
 **********************/
static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static void rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area);
static volatile bool g_lvgl_refr_enable = true;
static volatile bool g_lvgl_disp_enable = true;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_refresh_enable_set(bool enable)
{
    g_lvgl_refr_enable = enable;
}

bool lv_refresh_enable_get(void)
{
    return g_lvgl_refr_enable;
}

void lv_display_enable_set(bool enable)
{
    g_lvgl_disp_enable = enable;
}

bool lv_display_enable_get(void)
{
    return g_lvgl_disp_enable;
}

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create two fixed frame buffers for drawing and never release it
     *----------------------------*/
    static lv_disp_draw_buf_t draw_buf_dsc;
    lv_color_t* _draw_buf1 = app_graphics_mem_malloc(DISP_HOR_RES * DISP_HOR_RES * 2);
    lv_color_t* _draw_buf2 = app_graphics_mem_malloc(DISP_HOR_RES * DISP_HOR_RES * 2);
    lv_disp_draw_buf_init(&draw_buf_dsc, _draw_buf1, _draw_buf2, DISP_HOR_RES * DISP_HOR_RES);

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/
    static lv_disp_drv_t disp_drv;                  /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_HOR_RES;
    disp_drv.full_refresh = 1;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc;

    disp_drv.rounder_cb = &rounder_cb;

    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* Initialize your display and the required peripherals. */
static void disp_init(void)
{
    gx_dc_init();
}

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_disp_flush_ready()' has to be called when finished. */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(lv_display_enable_get())
    {
        gx_dc_flush(disp_drv, area, color_p);
    }
    disp_drv->draw_buf->flushing = 0;
    disp_drv->draw_buf->flushing_last = 0;
}

static void rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
  /* Per RM69330 datasheet, start coord and size must be even*/
    area->x1 = area->x1 & ~1;
    area->y1 = area->y1 & ~1;

    if ((area->x2 - area->x1 + 1) & 1) {
        area->x2 = area->x2 + 1;
    }
    if ((area->y2 - area->y1 + 1) & 1) {
        area->y2 = area->y2 + 1;
    }
}

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
