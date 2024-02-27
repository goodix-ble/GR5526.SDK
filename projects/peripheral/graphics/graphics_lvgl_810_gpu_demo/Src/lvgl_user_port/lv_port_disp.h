#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#ifdef __cplusplusW
extern "C" {
#endif

#include "lvgl.h"

#if (LV_COLOR_DEPTH != 32) || (GR552X_GPU_RENDER_SUPPORT == 0)
#error "GR5526 Only supported LV_COLOR_DEPTH=32 and GR552X_GPU_RENDER_SUPPORT=1 configuration"
#endif

#define GX_FRAME_SIZE  (DISP_HOR_RES * DISP_VER_RES * (2))

void lv_port_disp_init(void);

void lv_refresh_enable_set(bool enable);

bool lv_refresh_enable_get(void);

void lv_display_enable_set(bool enable);

bool lv_display_enable_get(void);

void lv_port_debug_info_enable(bool enable);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

