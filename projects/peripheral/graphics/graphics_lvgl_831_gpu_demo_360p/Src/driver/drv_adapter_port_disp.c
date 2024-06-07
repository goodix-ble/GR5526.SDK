#include "drv_adapter_port.h"
#include "graphics_dc_lcd_drv.h"
#include "graphics_dc_st77916_qspi_drv.h"

/***********************************************************************************
 *                 Static Declarations For Display
 ***********************************************************************************/
static void _disp_drv_init(disp_drv_t *dev);
static void _disp_drv_deinit(disp_drv_t *dev);
static void _disp_drv_set_show_area(disp_drv_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void _disp_drv_flush(disp_drv_t *dev, void *buff, uint32_t buff_format, uint16_t w, uint16_t h);
static void _disp_drv_wait_te(disp_drv_t *dev);
static void _disp_drv_wait_to_flush(disp_drv_t *dev);
static void _disp_drv_on(disp_drv_t *dev, bool on);
static void _disp_drv_set_brightness(disp_drv_t *dev, uint32_t percent);
static void _disp_drv_sleep(disp_drv_t *dev);
static void _disp_drv_wakeup(disp_drv_t *dev);

/***********************************************************************************
 *                 Public Implements
 ***********************************************************************************/

void drv_adapter_disp_register(void)
{
    disp_drv_t _disp_dev = {
        .idx = 0,
        .dev_id = 0,
        .user_data = NULL,
        .disp_drv_init = _disp_drv_init,
        .disp_drv_deinit = _disp_drv_deinit,
        .disp_drv_set_show_area = _disp_drv_set_show_area,
        .disp_drv_flush = _disp_drv_flush,
        .disp_drv_wait_te = _disp_drv_wait_te,
        .disp_drv_on = _disp_drv_on,
        .disp_drv_set_brightness = _disp_drv_set_brightness,
        .disp_drv_wait_to_flush = _disp_drv_wait_to_flush,
        .disp_drv_sleep = _disp_drv_sleep,
        .disp_drv_wakeup = _disp_drv_wakeup,
    };

    drv_adapter_disp_reg(0, &_disp_dev);

    return;
}

/***********************************************************************************
 *                 Static Implements for Display
 ***********************************************************************************/

static void _disp_drv_init(disp_drv_t *dev)
{
    (void)dev;
    graphics_dc_st77916_init(360, 360);
}

static void _disp_drv_set_show_area(disp_drv_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    (void)dev;
    graphics_dc_st77916_set_show_area(x1, x2, y1, y2);
}

static void _disp_drv_deinit(disp_drv_t *dev)
{
    graphics_dc_st77916_deinit();
}

static void _disp_drv_flush(disp_drv_t *dev, void *buff, uint32_t buff_format, uint16_t w, uint16_t h)
{
    graphics_dc_st77916_flush(buff, buff_format, w, h);
}

static void _disp_drv_wait_te(disp_drv_t *dev)
{
    (void)dev;
    graphics_dc_st77916_wait_te();
}

static void _disp_drv_wait_to_flush(disp_drv_t *dev)
{
    (void)dev;
    graphics_dc_st77916_wait_ready();
}

static void _disp_drv_on(disp_drv_t *dev, bool on)
{
    (void)dev;
    printf("Set Disp %s \r\n", on ? "ON" : "OFF");
    graphics_dc_st77916_set_on(on);
}

static void _disp_drv_set_brightness(disp_drv_t *dev, uint32_t percent)
{
    (void)dev;
    printf("Set Disp Brightness %d \r\n", percent);
    graphics_dc_st77916_set_brightness(percent);
}

static void _disp_drv_sleep(disp_drv_t *dev)
{
    (void)dev;
    graphics_dc_st77916_sleep();
}

static void _disp_drv_wakeup(disp_drv_t *dev)
{
    (void)dev;
    graphics_dc_st77916_wakeup();
}
