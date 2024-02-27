#ifndef __GX_HOME_LAYOUT_H__
#define __GX_HOME_LAYOUT_H__

#include "lvgl.h"
#include "lv_wms.h"

/* screen object variables */
extern lv_create_func_t g_cur_clock_view_create_func;
extern lv_create_func_t g_cur_list_view_create_func;

/* screen object create functions with WMS feature */
lv_obj_t* lv_flower_clock_layout_create(void);
lv_obj_t* lv_black_clock_layout_create(void);
lv_obj_t* lv_mars_clock_layout_create(void);
lv_obj_t* lv_activity_layout_create(void);
lv_obj_t* lv_heartrate_layout_create(void);
lv_obj_t* lv_chart_layout_create(void);
lv_obj_t* lv_status_bar_layout_create(void);
lv_obj_t* lv_message_layout_create(void);
lv_obj_t* lv_nebula_layout_create(void);
lv_obj_t* lv_cube_clock_layout_create(void);
lv_obj_t* lv_scale_list_layout_create(void);

/* screen object create functions without WMS feature */
lv_obj_t* lv_setting_view_create(void);
lv_obj_t* lv_list_view_select_create(void);
lv_obj_t* lv_clock_view_select_create(void);
lv_obj_t* lv_swtich_list_setting_view_create(void);
lv_obj_t* lv_volume_adjust_layout_create(void);
lv_obj_t* lv_graphics_circle_layout_create(void);
lv_obj_t* lv_graphics_cube_layout_create(void);
lv_obj_t* lv_graphics_arc_layout_create(void);
lv_obj_t* lv_graphics_jump_layout_create(void);
lv_obj_t* lv_graphics_stencil_layout_create(void);
lv_obj_t* lv_graphics_tsc4_layout_create(void);
lv_obj_t* lv_widgets_test_view_create(void);

/* screen object destroy functions */
void lv_flower_clock_layout_destroy(void);
void lv_black_clock_layout_destroy(void);
void lv_mars_clock_layout_destroy(void);
void lv_activity_layout_destroy(void);
void lv_heartrate_layout_destroy(void);
void lv_chart_layout_destroy(void);
void lv_status_bar_layout_destroy(void);
void lv_message_layout_destroy(void);
void lv_setting_view_destroy(void);
void lv_clock_view_select_destroy(void);
void lv_list_view_select_destroy(void);
void lv_i8_clock_layout_destroy(void);
void lv_nebula_layout_destroy(void);
void lv_cube_clock_layout_destroy(void);
void lv_scale_list_layout_destroy(void);
void lv_swtich_list_setting_view_destroy(void);
void lv_volume_adjust_layout_destroy(void);
void lv_graphics_circle_layout_destroy(void);
void lv_graphics_cube_layout_destroy(void);
void lv_graphics_arc_layout_destroy(void);
void lv_graphics_jump_layout_destroy(void);
void lv_graphics_stencil_layout_destroy(void);
void lv_graphics_tsc4_layout_destroy(void);
/* moniter the lv heap used size */
uint32_t lv_mem_used_size_get(void);
void lvgl_mem_used_dump(const char* func, int line);

#endif
