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
static lv_obj_t* setting_layout;
static const char setting_msg[][15]= 
{
    "应用视图 >",
    "过场效果 >",
    "调试模式 >",
    "语言设置 >",
    "系统菜单 >",
    "通用设置 >",
    "屏幕显示 >",
    "闹钟设置 >",
    "通知管理 >",
    "系统升级 >",
    "关于手表 >",
};

static void event_handler_1(lv_event_t* e);
static void event_handler_2(lv_event_t* e);
static void event_handler_3(lv_event_t* e);
static void event_handler_4(lv_event_t* e);
static void event_handler_5(lv_event_t* e);
static void event_handler_6(lv_event_t* e);
static void event_handler_7(lv_event_t* e);
static void event_handler_8(lv_event_t* e);
static void event_handler_9(lv_event_t* e);
static void event_handler_10(lv_event_t* e);
static void event_handler_11(lv_event_t* e);

typedef void (*event_handler_func)(lv_event_t* e);

void lv_setting_view_destroy(void)
{
    if(setting_layout == NULL) return;
    lv_wms_deinit(setting_layout);
    lv_obj_del(setting_layout);
    setting_layout = NULL;
}

const  lv_img_dsc_t*  setting_imgs[] = 
{
    &wd_img_app_layout, //ADDR_1_APP_LAYOUT,
    &wd_img_switch,
    &wd_img_debug, //ADDR_4_DEBUG,
    &wd_img_language,//ADDR_3_LANGUAGE,
    &wd_img_menu, //ADDR_5_MENU,
    &wd_img_settings, //ADDR_6_SETTINGS,
    &wd_img_lightness, //ADDR_7_LIGHTNESS,
    &wd_img_lightness,
    &wd_img_lightness,
    &wd_img_lightness,
    &wd_img_lightness,
};

static event_handler_func  event_array[] = {
    event_handler_1,
    event_handler_2,
    event_handler_3,
    event_handler_4,
    event_handler_5,
    event_handler_6,
    event_handler_7,
    event_handler_8,
    event_handler_9,
    event_handler_10,
    event_handler_11
};

static void event_handler_1(lv_event_t* e)
{
    lv_wms_go_to_window(lv_swtich_list_setting_view_create);
}

static void event_handler_2(lv_event_t* e)
{
    lv_wms_go_to_window(lv_transit_effect_setting_view_create);
}

static void event_handler_3(lv_event_t* e)
{
    lv_wms_go_to_window(lv_debug_mode_setting_view_create);
}

static void event_handler_4(lv_event_t* e)
{

}

static void event_handler_5(lv_event_t* e)
{

}

static void event_handler_6(lv_event_t* e)
{

}

static void event_handler_7(lv_event_t* e)
{

}

static void event_handler_8(lv_event_t* e)
{

}

static void event_handler_9(lv_event_t* e)
{

}

static void event_handler_10(lv_event_t* e)
{

}

static void event_handler_11(lv_event_t* e)
{

}

lv_obj_t* lv_setting_view_create(void)
{
    if(setting_layout == NULL){
        setting_layout = lv_obj_create(NULL);
        lv_obj_set_size(setting_layout, lv_pct(100), lv_pct(100));
        lv_obj_set_style_bg_color(setting_layout, lv_color_black(), 0);
        lv_obj_t* lab_set = lv_label_create(setting_layout);
        lv_obj_set_style_text_font(lab_set, &lv_font_msyhbd_40, LV_STATE_DEFAULT);
        lv_label_set_text_fmt(lab_set, "设置");
        lv_obj_set_style_text_color(lab_set, lv_color_white(), 0);
        lv_obj_set_pos(lab_set, 186, 50);
        lv_obj_t* set_list = lv_obj_create(setting_layout);
        lv_obj_set_scrollbar_mode(set_list, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(set_list, LV_DIR_VER);
        lv_obj_set_style_pad_row(set_list, 30, 0);
        lv_obj_clear_flag(set_list, LV_OBJ_FLAG_SCROLL_ELASTIC);
        lv_obj_set_size(set_list, 300, 300);
        lv_obj_set_pos(set_list, 70, 110);
        lv_obj_set_style_text_font(set_list, &lv_font_simsun_34, LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(set_list, lv_color_white(), LV_PART_ANY);
        lv_obj_set_style_border_width(set_list, 0, 0);

        for (uint32_t i = 0; i < 11; i++){
            lv_obj_t* obj = lv_obj_create(set_list);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scroll_dir(obj, LV_DIR_NONE);
            lv_obj_set_size(obj, 300, 70);
            lv_obj_set_pos(obj, 0, i * 80);
            lv_obj_t* img = lv_img_create(obj);
            lv_img_set_src(img, setting_imgs[i]);
            lv_obj_t* label = lv_label_create(obj);
            lv_label_set_text(label, setting_msg[i]);
            lv_obj_align_to(label, img, LV_ALIGN_LEFT_MID, 70, 0);
            lv_obj_add_event_cb(obj, event_array[i], LV_EVENT_CLICKED, NULL);
        }
    }

    lv_wms_init(setting_layout);
    lv_wms_self_destroy_func_set(setting_layout, lv_setting_view_destroy);
    lv_wms_left_create_func_set(setting_layout, g_cur_list_view_create_func);
    lv_scr_load(setting_layout);
    lvgl_mem_used_dump(__func__, __LINE__);
    return setting_layout;
}
