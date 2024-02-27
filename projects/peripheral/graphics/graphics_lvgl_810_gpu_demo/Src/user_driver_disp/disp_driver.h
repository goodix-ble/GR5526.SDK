#ifndef __GX_DISP_DRIVER_H__
#define __GX_DISP_DRIVER_H__

#include "lv_port_disp.h"

void gx_dc_init(void);

void gx_dc_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void gx_dc_flush_transition(void* addr, uint32_t w, uint32_t h, uint32_t format);

void gx_dc_screen_off(void);

void gx_dc_screen_on(void);
#endif
