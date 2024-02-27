/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "lv_port_disp.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t* g_activity_view = NULL;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t * arc;
static lv_obj_t * arc_middle;
static lv_obj_t * arc_big;
static lv_obj_t * lable;
static lv_obj_t * lable_middle;
static lv_obj_t * lable_big;
static lv_anim_t anim;

static void set_angle(void * obj, int32_t v)
{
    if(NULL == g_activity_view) return;
    uint32_t value = v * 97 / 60;
    uint32_t value_mid = v * 6597 / 60;
    uint32_t value_big = v * 15978 / 60;

    lv_arc_set_value(obj, v);
    lv_arc_set_value(arc_middle, v);
    lv_arc_set_value(arc_big, v);

    lv_label_set_text_fmt(lable, "%d", value);
    lv_label_set_text_fmt(lable_middle, "%d", value_mid);
    lv_label_set_text_fmt(lable_big, "%d", value_big);
}

void lv_activity_layout_destroy(void)
{
    if(NULL == g_activity_view) return;
    lv_wms_deinit(g_activity_view);
    lv_obj_del(g_activity_view);
    g_activity_view = NULL;
}

lv_obj_t* lv_activity_layout_create(void)
{
    if(NULL != g_activity_view) return g_activity_view;

    g_activity_view = lv_obj_create(NULL);
    lv_obj_t* obj = g_activity_view;
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);

    /*Create an Arc*/
    arc = lv_arc_create(obj);
    lv_arc_set_rotation(arc, 135);
    lv_obj_set_size(arc, 180, 180);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_obj_set_style_arc_width(arc, 30, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc);

    arc_middle = lv_arc_create(obj);
    lv_arc_set_rotation(arc_middle, 135);
    lv_obj_set_size(arc_middle, 280, 280);
    lv_arc_set_bg_angles(arc_middle, 0, 270);
    //lv_arc_set_value(arc_middle, 70);
    lv_obj_set_style_arc_width(arc_middle, 30, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc_middle, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc_middle, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_INDICATOR);
    lv_obj_remove_style(arc_middle, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(arc_middle, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc_middle);

    arc_big = lv_arc_create(obj);
    lv_arc_set_rotation(arc_big, 135);
    lv_obj_set_size(arc_big, 380, 380);
    lv_arc_set_bg_angles(arc_big, 0, 270);
    lv_obj_set_style_arc_width(arc_big, 30, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc_big, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc_big, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
    lv_obj_remove_style(arc_big, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(arc_big, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc_big);

    lable = lv_label_create(obj);
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(lable, lv_palette_main(LV_PALETTE_RED), 0);
    lv_label_set_text(lable, "97");
    lv_label_set_long_mode(lable, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_pos(lable, 195, 280);

    lable_middle = lv_label_create(obj);
    lv_obj_set_style_text_color(lable_middle, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_label_set_text(lable_middle, "6597");
    lv_obj_set_pos(lable_middle, 180, 330);

    lable_big = lv_label_create(obj);
    lv_obj_set_style_text_color(lable_big, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_label_set_text(lable_big, "15978");
    lv_obj_set_pos(lable_big, 170, 380);

    lv_obj_t* img1 = lv_img_create(obj);
    lv_img_set_src(img1, &wd_img_digital2_heart_small);
    lv_obj_set_pos(img1, 160, 270);

    lv_obj_t* img3 = lv_img_create(obj);
    lv_img_set_src(img3, &wd_img_digital2_consume_small);
    lv_obj_set_pos(img3, 124, 300);

    lv_obj_t* img2 = lv_img_create(obj);
    lv_img_set_src(img2, &wd_img_digital2_exercise_small);
    lv_obj_set_pos(img2, 88, 335);

    lv_anim_init(&anim);
    lv_anim_set_var(&anim, arc);
    lv_anim_set_exec_cb(&anim, set_angle);
    lv_anim_set_time(&anim, 1000);
    lv_anim_set_repeat_delay(&anim, 500);
    lv_anim_set_values(&anim, 0, 60);
    lv_anim_start(&anim);

    lv_wms_init(g_activity_view);
    lv_wms_self_destroy_func_set(g_activity_view, lv_activity_layout_destroy);
    lv_wms_left_create_func_set(g_activity_view, g_cur_clock_view_create_func);
    //lv_wms_right_create_func_set(g_activity_view, lv_heartrate_layout_create);
    lv_wms_right_create_func_set(g_activity_view, lv_chart_layout_create);

    lv_scr_load(g_activity_view);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_activity_view;
}
