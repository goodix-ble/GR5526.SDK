/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"
#include "lv_setting_layout.h"
#include "lv_switch_effect_layout.h"
#include "lv_layout_manager.h"
#include "lv_wms_surface_flinger.h"
#include "lv_user_font.h"
#include "lv_scale_list.h"
#include "lv_custom_obj.h"
#include "lv_graphics_draw.h"
/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* graphics_tsc4_layout = NULL;

void lv_graphics_tsc4_layout_destroy(void){
    if(NULL == graphics_tsc4_layout) return;
    lv_wms_deinit(graphics_tsc4_layout);
    lv_obj_del(graphics_tsc4_layout);
    graphics_tsc4_layout = NULL;
}

lv_obj_t* lv_graphics_tsc4_layout_create(void)
{
    if (graphics_tsc4_layout == NULL){

        graphics_tsc4_layout = lv_obj_create(NULL);
        lv_obj_t* graphics_circle = lv_graphics_draw_obj_create(graphics_tsc4_layout);
        lv_set_graphics_draw_type(GRAPHICS_TYPE_TSC4_COMP);
        lv_obj_set_size(graphics_circle, 454, 454);
    }
    lv_wms_init(graphics_tsc4_layout);
    lv_wms_self_destroy_func_set(graphics_tsc4_layout, lv_graphics_tsc4_layout_destroy);
    lv_wms_left_create_func_set(graphics_tsc4_layout, g_cur_list_view_create_func);

    lv_scr_load(graphics_tsc4_layout);
    lvgl_mem_used_dump(__func__, __LINE__);
    return graphics_tsc4_layout;
}

