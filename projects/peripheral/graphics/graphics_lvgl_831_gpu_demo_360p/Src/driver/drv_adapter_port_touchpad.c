#include "drv_adapter_port.h"
#include "tp_cst816d_driver.h"

/***********************************************************************************
 *                 Static Declarations For Nor-Flash
 ***********************************************************************************/
static bool _touchpad_drv_init(touchpad_drv_t *dev);
static bool _touchpad_drv_deinit(touchpad_drv_t *dev);
static bool _touchpad_drv_read_pointer(touchpad_drv_t *dev, int16_t *x, int16_t *y);
static bool _touchpad_drv_sleep(touchpad_drv_t *dev);
static bool _touchpad_drv_wakeup(touchpad_drv_t *dev);

/***********************************************************************************
 *                 Public Implements
 ***********************************************************************************/



void drv_adapter_touchpad_register(void)
{
    touchpad_drv_t _tp_dev = {
        .id = 0,
        .touchpad_drv_init = _touchpad_drv_init,
        .touchpad_drv_deinit = _touchpad_drv_deinit,
        .touchpad_drv_read_pointer = _touchpad_drv_read_pointer,
        .touchpad_drv_sleep = _touchpad_drv_sleep,
        .touchpad_drv_wakeup = _touchpad_drv_wakeup,
    };

    drv_adapter_touchpad_reg(&_tp_dev);
}

__weak void _touchpad_drv_irq_notify(void)
{
}

/***********************************************************************************
 *                 Static Implements for Nor-Flash
 ***********************************************************************************/

static bool _touchpad_drv_init(touchpad_drv_t *dev)
{
    (void)dev;
    tp_cst816d_init(_touchpad_drv_irq_notify);
    return true;
}

static bool _touchpad_drv_deinit(touchpad_drv_t *dev)
{
    (void)dev;
    tp_cst816d_deinit();
    return true;
}

static bool _touchpad_drv_read_pointer(touchpad_drv_t *dev, int16_t *x, int16_t *y)
{
    (void)dev;
    return tp_cst816d_read_pointer(x, y);
}

static bool _touchpad_drv_sleep(touchpad_drv_t *dev)
{
    (void)dev;
    return tp_cst816d_sleep();
}

static bool _touchpad_drv_wakeup(touchpad_drv_t *dev)
{
    (void)dev;
    return tp_cst816d_wakeup();
}
