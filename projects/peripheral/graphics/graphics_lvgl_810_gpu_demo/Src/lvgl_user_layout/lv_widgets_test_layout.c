/*
 * README
 *****************************************************************************************
 * This layout is used to test kinds of lvgl widgets
 *
 */

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
#include "lv_user_font.h"
/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_msyhbd_40;
extern lv_font_t lv_font_msyhbd_34;
extern lv_obj_t* lv_debug_mode_setting_view_create(void);
extern void lv_debug_mode_setting_view_destroy(void);

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* _widgets_test_layout;

void lv_widgets_test_view_destroy(void)
{
    if(_widgets_test_layout == NULL) return;
    lv_wms_deinit(_widgets_test_layout);
    lv_obj_del(_widgets_test_layout);
    _widgets_test_layout = NULL;
}

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        printf("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
    }
}

lv_obj_t* lv_widgets_test_view_create(void)
{
    //lv_obj_set_flex_flow(lv_scr_act(), LV_FLEX_FLOW_COLUMN);
    //lv_obj_set_flex_align(lv_scr_act(), LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    //lv_obj_t * _widgets_test_layout;

    if(NULL == _widgets_test_layout) {


        _widgets_test_layout = lv_obj_create(NULL);
        lv_obj_set_size(_widgets_test_layout, 454, 454);
        lv_obj_set_style_bg_color(_widgets_test_layout, lv_palette_main(LV_PALETTE_GREY), 0);


        static lv_style_t style_volume;
        lv_style_reset(&style_volume);
        lv_style_init(&style_volume);
        lv_style_set_bg_opa(&style_volume, LV_OPA_COVER);
        lv_style_set_bg_color(&style_volume, lv_palette_main(LV_PALETTE_BLUE));

        //lv_obj_t* sw = lv_switch_create(_widgets_test_layout);
        lv_obj_t* sw = lv_checkbox_create(_widgets_test_layout);
        lv_checkbox_set_text(sw, "Apple");

        lv_obj_add_style(sw, &style_volume, LV_PART_MAIN);
        lv_obj_set_size(sw, (64), (64));
        lv_obj_set_style_bg_img_src(sw, LV_SYMBOL_GPS /*&wd_img_37_QQ_music*/, 0);

        lv_obj_set_style_text_font(sw, &lv_font_symbol_40, LV_STATE_DEFAULT);

        lv_obj_center(sw);
        //lv_obj_set_size(sw, 100, 40);
        lv_obj_add_state(sw, LV_STATE_CHECKED | LV_STATE_DISABLED);
        lv_obj_add_event_cb(sw, event_handler, LV_EVENT_ALL, NULL);

//        sw = lv_switch_create(_widgets_test_layout);
//        lv_obj_add_state(sw, LV_STATE_CHECKED);
//        lv_obj_add_event_cb(sw, event_handler, LV_EVENT_ALL, NULL);

//        sw = lv_switch_create(_widgets_test_layout);
//        lv_obj_add_state(sw, LV_STATE_DISABLED);
//        lv_obj_add_event_cb(sw, event_handler, LV_EVENT_ALL, NULL);

//        sw = lv_switch_create(_widgets_test_layout);
//        lv_obj_add_state(sw, LV_STATE_CHECKED | LV_STATE_DISABLED);
//        lv_obj_add_event_cb(sw, event_handler, LV_EVENT_ALL, NULL);
    }

    lv_wms_init(_widgets_test_layout);
    lv_wms_self_destroy_func_set(_widgets_test_layout, lv_widgets_test_view_destroy);
    lv_wms_left_create_func_set(_widgets_test_layout, g_cur_list_view_create_func);
    lv_scr_load(_widgets_test_layout);
    lvgl_mem_used_dump(__func__, __LINE__);
    return _widgets_test_layout;
}

