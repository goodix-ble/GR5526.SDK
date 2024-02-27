/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "lv_port_disp.h"

/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_simsun_28;

/*
 * DEFINE
 *****************************************************************************************
 */
static lv_obj_t* heart_img_obj = NULL;
static lv_timer_t* heart_timer = NULL;
static lv_obj_t * chart;
static lv_chart_series_t * ser1 = NULL;
lv_obj_t* g_heartrate_view = NULL;

#define CHAR_VALUE_STEP   4
#define CHAR_VALUE_MAX    26
#define CHAR_VALUE_MIN    0
#define CHAR_NB           24

const int32_t chart_init_value[CHAR_NB] = {8, 10, 12, 15, 18, 22, \
                         26, 22, 18, 14, 10, 8, \
                         8, 10, 12, 15, 18, 22, \
                         26, 22, 18, 14, 10, 8};

int32_t chart_value[CHAR_NB] = {2, 6, 10, 14, 18, 22, \
                         26, 22, 18, 14, 10, 6, \
                         2, 6, 10, 14, 18, 22, \
                         26, 22, 18, 14, 10, 6};

static int32_t chart_value_step[CHAR_NB] = {0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0};

static void lv_layout_timer(lv_timer_t * tmr)
{
    if(NULL == g_heartrate_view) return;
    static uint32_t pos = 360;
    static int32_t  pos_change = -6;
    for (uint32_t i = 0; i < 24; i++)
    {
        lv_chart_set_next_value(chart, ser1, chart_value[i]);
        chart_value[i] += chart_value_step[i];
        if (chart_value[i] >= CHAR_VALUE_MAX)
        {
            chart_value[i] = CHAR_VALUE_MAX;
            chart_value_step[i] = -CHAR_VALUE_STEP;
        }
        if (chart_value[i] <= CHAR_VALUE_MIN)
        {
            chart_value[i] = CHAR_VALUE_MIN;
            chart_value_step[i] = CHAR_VALUE_STEP;
        }
    }
    lv_obj_set_pos(heart_img_obj, 100, pos);
    pos += pos_change;
    if (pos <= 300){
        pos_change = 10;
    }
    if (pos >= 360){
        pos_change = -10;
    }
}

static void draw_event_cb(lv_event_t * e)
{
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if(!lv_obj_draw_part_check_type(dsc, &lv_chart_class, LV_CHART_DRAW_PART_TICK_LABEL)) return;

    if(dsc->id == LV_CHART_AXIS_PRIMARY_X && dsc->text) {
        const char * month[] = {"0", "2", "4", "6", "8", "10", "12", "14", "16", "18", "20", "22", "24"};
        lv_snprintf(dsc->text, dsc->text_length, "%s", month[dsc->value]);
    }
}

void lv_heartrate_layout_destroy(void)
{
    if(NULL == g_heartrate_view) return;
    lv_timer_del(heart_timer);
    heart_timer = NULL;
    lv_wms_deinit(g_heartrate_view);
    lv_obj_del(g_heartrate_view);
    g_heartrate_view = NULL;
}

lv_obj_t* lv_heartrate_layout_create(void)
{
    if(NULL != g_heartrate_view) return g_heartrate_view;
    /* build screen object */
    g_heartrate_view = lv_obj_create(NULL);
    lv_obj_set_scrollbar_mode(g_heartrate_view, LV_SCROLLBAR_MODE_OFF);

    /*Create a chart*/
    chart = lv_chart_create(g_heartrate_view);
    lv_obj_set_style_text_font(chart, &lv_font_montserrat_8, LV_STATE_DEFAULT);
    lv_obj_set_size(chart, 300, 190);
    lv_obj_set_pos(chart, 76, 80);
    lv_chart_set_type(chart, LV_CHART_TYPE_BAR);
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 30);
    lv_chart_set_point_count(chart, 24);
    lv_obj_add_event_cb(chart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_obj_set_style_bg_color(chart, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(chart, 0, LV_PART_MAIN);

    /*Add ticks and label to every axis*/
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 13, 2, true, 40);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 6, 2, true, 50);
    lv_chart_set_div_line_count(chart, 5, 0);
    lv_obj_set_style_pad_column(chart, 1, LV_PART_MAIN);

    /*Add two data series*/
    ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_SECONDARY_Y);
    memset(chart_value_step, 0, CHAR_NB*sizeof(int32_t));
    memcpy(chart_value, chart_init_value, CHAR_NB*sizeof(int32_t));
    for (uint32_t i = 0; i < 24; i++)
    {
        chart_value_step[i] = CHAR_VALUE_STEP;
        if (chart_value[i] >= CHAR_VALUE_MAX)
        {
            chart_value_step[i] = -CHAR_VALUE_STEP;
        }
        if (chart_value[i] <= CHAR_VALUE_MIN)
        {
            chart_value_step[i] = CHAR_VALUE_STEP;            
        }
        lv_chart_set_next_value(chart, ser1, chart_value[i]);
        chart_value[i] += chart_value_step[i];
        if (chart_value[i] >= CHAR_VALUE_MAX)
        {
            chart_value[i] = CHAR_VALUE_MAX;
            chart_value_step[i] = -CHAR_VALUE_STEP;
        }
        if (chart_value[i] <= CHAR_VALUE_MIN)
        {
            chart_value[i] = CHAR_VALUE_MIN;
            chart_value_step[i] = CHAR_VALUE_STEP;            
        }
    }
    lv_chart_refresh(chart); /*Required after direct set*/

    heart_img_obj = lv_img_create(g_heartrate_view);
    lv_img_set_src(heart_img_obj, &wd_img_heartrate1);
    lv_obj_set_pos(heart_img_obj, 100, 360);

    lv_obj_t* label = lv_label_create(g_heartrate_view);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_40, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text_fmt(label, "%d", 80);
    lv_obj_set_pos(label, 200, 330);

    lv_obj_t* label2 = lv_label_create(g_heartrate_view);
    lv_obj_set_style_text_font(label2, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label2, lv_palette_main(LV_PALETTE_RED), 0);
    lv_label_set_text_fmt(label2, "次/分");
    lv_obj_set_pos(label2, 270, 340);

    if(NULL == heart_timer){
        heart_timer = lv_timer_create(lv_layout_timer, 100, NULL);
        lv_timer_resume(heart_timer);
    }

    lv_wms_init(g_heartrate_view);
    lv_wms_self_destroy_func_set(g_heartrate_view, lv_heartrate_layout_destroy);
    //lv_wms_left_create_func_set(g_heartrate_view, lv_activity_layout_create);
    lv_wms_left_create_func_set(g_heartrate_view, lv_chart_layout_create);
    lv_wms_right_create_func_set(g_heartrate_view, g_cur_list_view_create_func);
    lv_scr_load(g_heartrate_view);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_heartrate_view;
}
