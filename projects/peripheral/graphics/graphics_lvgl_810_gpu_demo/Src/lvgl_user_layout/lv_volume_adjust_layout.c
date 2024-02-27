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
/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* volume_adjust_layout = NULL;

void lv_volume_adjust_layout_destroy(void){
    if(NULL == volume_adjust_layout) return;
    lv_wms_deinit(volume_adjust_layout);
    lv_obj_del(volume_adjust_layout);
    volume_adjust_layout = NULL;
}

static void event_handler(lv_event_t* e)
{
    static lv_point_t act_point;
    lv_indev_t * indev_act;
    lv_obj_draw_part_dsc_t * dsc;
    lv_area_t clip_area;

    switch(e->code){
        case LV_EVENT_PRESSING:
            indev_act = lv_indev_get_act();
            act_point = indev_act->proc.types.pointer.act_point;
            lv_obj_invalidate(e->current_target);
            lv_obj_clear_flag(e->current_target, LV_OBJ_FLAG_EVENT_BUBBLE);
            break;

        case LV_EVENT_DRAW_PART_BEGIN:
            dsc = lv_event_get_draw_part_dsc(e);
            memcpy(&clip_area, dsc->clip_area, sizeof(lv_area_t));
            if (act_point.y != 0){
                clip_area.y1 = act_point.y;
            }
            lv_custom_obj_set_clip_area(clip_area);
            break;
        case LV_EVENT_RELEASED:
            lv_obj_add_flag(e->current_target, LV_OBJ_FLAG_EVENT_BUBBLE);
            break;
        default:
            break;
    }
}

lv_obj_t* lv_volume_adjust_layout_create(void)
{
    if (volume_adjust_layout == NULL){

        static lv_style_t style_volume;
        lv_style_reset(&style_volume);
        lv_style_init(&style_volume);
        lv_style_set_bg_opa(&style_volume, LV_OPA_COVER);
        lv_style_set_bg_color(&style_volume, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_radius(&style_volume, 40);

        volume_adjust_layout = lv_obj_create(NULL);
        lv_obj_t* volume_bg = lv_obj_create(volume_adjust_layout);
        lv_obj_add_style(volume_bg, &style_volume, LV_PART_MAIN);
        lv_obj_center(volume_bg);
        lv_obj_set_size(volume_bg, 80, 250);
        lv_obj_add_style(volume_bg, &style_volume, LV_PART_MAIN);
        lv_obj_set_style_bg_color(volume_bg, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
        lv_obj_t * volume_obj = lv_custom_obj_create(volume_adjust_layout);
        lv_obj_center(volume_obj);
        lv_obj_set_size(volume_obj, 80, 250);
        lv_obj_add_style(volume_obj, &style_volume, LV_PART_MAIN);
        lv_obj_add_event_cb(volume_obj, event_handler, LV_EVENT_PRESSING, NULL);
        lv_obj_add_event_cb(volume_obj, event_handler, LV_EVENT_DRAW_PART_BEGIN, NULL);
        lv_obj_add_event_cb(volume_obj, event_handler, LV_EVENT_RELEASED, NULL);

        lv_obj_t* img2 = lv_img_create(volume_obj);
        lv_img_set_src(img2, &wd_img_digital2_exercise_small);
        lv_obj_align(img2, LV_ALIGN_BOTTOM_MID, 0, -20);
    }
    lv_wms_init(volume_adjust_layout);
    lv_wms_self_destroy_func_set(volume_adjust_layout, lv_volume_adjust_layout_destroy);
    lv_wms_left_create_func_set(volume_adjust_layout, g_cur_list_view_create_func);

    lv_scr_load(volume_adjust_layout);
    lvgl_mem_used_dump(__func__, __LINE__);
    return volume_adjust_layout;
}

