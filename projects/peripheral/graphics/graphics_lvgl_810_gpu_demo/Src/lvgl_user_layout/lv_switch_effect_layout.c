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
/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_msyhbd_40;

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* switch_layout;

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED){
        char buf[32];
        lv_roller_get_selected_str(obj, buf, sizeof(buf));
        // TBD with new effect setting module
        printf("effect: %d\n", lv_roller_get_selected(obj));

        switch(lv_roller_get_selected(obj)) {
            case 0:
            {
                lv_wms_transit_effect_set((lv_transit_effect_e) LV_TRANS_EFFECT_CUBE);
            }
            break;

            case 1:
            {
                lv_wms_transit_effect_set((lv_transit_effect_e) LV_TRANS_EFFECT_LINEAR);            
            }
            break;

            default:
            {
                lv_wms_transit_effect_set((lv_transit_effect_e) lv_roller_get_selected(obj));
            }
            break;
        }
    }
}

void lv_transit_effect_setting_view_destroy(void){
    if(NULL == switch_layout) return;
    lv_wms_deinit(switch_layout);
    lv_obj_del(switch_layout);
    switch_layout = NULL;
}

lv_obj_t* lv_transit_effect_setting_view_create(void)
{
    if(NULL == switch_layout){
        switch_layout = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(switch_layout, lv_color_black(), 0);
        lv_obj_set_size(switch_layout, lv_pct(100), lv_pct(100));

        lv_obj_t* lab = lv_label_create(switch_layout);
        lv_obj_set_style_text_font(lab, &lv_font_msyhbd_40, LV_STATE_DEFAULT);
        lv_label_set_text_fmt(lab, "过场效果");
        lv_obj_set_style_text_color(lab, lv_color_white(), 0);
        lv_obj_align(lab, LV_ALIGN_TOP_MID, 0, 50);
        //lv_obj_set_pos(lab, 146, 50);

        lv_obj_t* roller1 = lv_roller_create(switch_layout);
        lv_obj_set_style_bg_color(roller1, lv_color_black(), 0);
        lv_obj_set_pos(roller1, 0, 100);
        lv_obj_set_size(roller1, 454, 250);

        lv_roller_set_options(roller1,
                            "立方变换\n"
                            "线性过渡\n"
                            "立方拉伸\n"
                            "堆栈转换\n"
                            "渐变过渡\n"
                            "淡入淡出\n"
                            "线性覆盖",
                            LV_ROLLER_MODE_NORMAL);
        lv_wms_transit_effect_set((lv_transit_effect_e) LV_TRANS_EFFECT_CUBE);

        static lv_style_t style = {0};
        lv_style_set_text_color(&style, lv_palette_lighten(LV_PALETTE_GREY, 1));
        lv_style_set_text_font(&style, &lv_font_simsun_34);
        lv_style_set_text_line_space(&style, 60);
        lv_style_set_border_width(&style, 0);
        lv_obj_add_style(roller1, &style, LV_PART_MAIN);
        lv_roller_set_visible_row_count(roller1, 3);
        static lv_style_t style_select;
        lv_style_set_text_font(&style_select, &lv_font_msyhbd_40);
        lv_style_set_bg_color(&style_select, lv_color_make(0x19,0x19,0x19));
        lv_style_set_text_color(&style_select, lv_palette_main(LV_PALETTE_BLUE));
        lv_obj_add_style(roller1, &style_select, LV_PART_SELECTED);
        lv_obj_add_event_cb(roller1, event_handler, LV_EVENT_ALL, NULL);
    }

    lv_wms_init(switch_layout);
    lv_wms_self_destroy_func_set(switch_layout, lv_transit_effect_setting_view_destroy);
    lv_wms_left_create_func_set(switch_layout, lv_setting_view_create);

    lv_scr_load(switch_layout);
    lvgl_mem_used_dump(__func__, __LINE__);
    return switch_layout;
}
