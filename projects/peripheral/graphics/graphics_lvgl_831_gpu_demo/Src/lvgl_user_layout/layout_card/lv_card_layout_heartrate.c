#include <stdio.h>
#include "lvgl.h"
#include "lv_layout_router.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"

#define HR_TITLE_X (146)
#define HR_TITLE_Y (30)
#define HR_ICON_X (120)
#define HR_ICON_Y (72)
#define HR_VAL_X (196)
#define HR_VAL_Y (76)
#define HR_UNIT_X (268)
#define HR_UNIT_Y (86)
#define HR_MEAS_TIME_X (156)
#define HR_MEAS_TIME_Y (136)

#define HR_CHART_LEFT_PAD (6)
#define HR_CHART_BOTTOM_PAD (28)
#define HR_CHART_X_SPACING (15)
#define HR_CHART_X (40)
#define HR_CHART_Y (176)
#define HR_CHART_Y_MAX_TICKS_X (396)
#define HR_CHART_Y_MAX_TICKS_Y (176)
#define HR_CHART_Y_MIN_TICKS_X (396)
#define HR_CHART_Y_MIN_TICKS_Y (300)

#define HR_MAX_ICON_X (122)
#define HR_MAX_ICON_Y (374)
#define HR_MAX_VAL_X (146)
#define HR_MAX_VAL_Y (366)
#define HR_MIN_ICON_X (270)
#define HR_MIN_ICON_Y (374)
#define HR_MIN_VAL_X (294)
#define HR_MIN_VAL_Y (366)

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */

static lv_obj_t *_heart_meas_time = NULL;
static lv_obj_t *_heart_val = NULL;
static lv_obj_t *_heart_max = NULL;
static lv_obj_t *_heart_min = NULL;
static lv_obj_t *_heart_val_line[24] = {NULL};
static lv_point_t _heart_val_pts[24][2];
static lv_obj_t *_heart_chart;
static lv_obj_t *_heart_chart_max_ticks;
static lv_obj_t *_heart_chart_min_ticks;
static lv_timer_t *_hb_animation_timer = NULL;

static uint8_t hr_max_data[] =
    {90, 97, 80, 100, 97, 88,
     110, 102, 121, 119, 107, 127,
     109, 102, 120, 107, 99, 131,
     117, 127, 90, 97, 87, 101};

static uint8_t hr_min_data[] =
    {40, 48, 40, 50, 43, 44,
     55, 51, 64, 60, 53, 60,
     54, 51, 60, 57, 46, 77,
     56, 67, 60, 57, 40, 51};

/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
static uint32_t _hr_val_to_y_pos(uint8_t hr_val)
{
    uint32_t ret = HR_CHART_Y + wd_img_table_bg_scaled.header.h - HR_CHART_BOTTOM_PAD - hr_val / 220.f * (wd_img_table_bg_scaled.header.h - HR_CHART_BOTTOM_PAD);
    return ret;
}

static void _hr_set_max_min_val(uint8_t *p_max_data, uint8_t *p_min_data)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        _heart_val_pts[i][0].x = HR_CHART_X + HR_CHART_LEFT_PAD + i * HR_CHART_X_SPACING;
        _heart_val_pts[i][0].y = _hr_val_to_y_pos(p_max_data[i]);
        _heart_val_pts[i][1].x = _heart_val_pts[i][0].x;
        _heart_val_pts[i][1].y = _hr_val_to_y_pos(p_min_data[i]);
        lv_line_set_points(_heart_val_line[i], _heart_val_pts[i], 2);
    }
}

static void _hb_timer_callback(lv_timer_t *tmr)
{

}

static lv_obj_t * p_container_img = NULL;
static lv_obj_t * p_container_layout = NULL;

static lv_img_dsc_t* p_snapshot = NULL;






static uint32_t user_data = 10;

static lv_timer_t * p_container_timer = NULL;
static bool is_timer_running   = false;
static int  p_container_degree = 0;
static int  p_degree_cnt       = 0;               //0-[0,90], 1-[90, 180], 2-[180,270], 3-[270,360] 
static int  p_degree_delta     = 0;

void rotate_timer(lv_timer_t * timer)
{
    /*Use the user_data*/
    uint32_t * user_data = timer->user_data;
    printf("my_timer called with user data: %d\n", *user_data);

    p_degree_delta ++;
    p_degree_delta = p_degree_delta % 10;
    
    if(p_degree_delta == 0) {
        lv_timer_pause(p_container_timer);
        p_degree_cnt ++;
        p_degree_cnt = p_degree_cnt % 4;
        
        is_timer_running = false;
        
        return;
    }
    
    lv_img_set_src(p_container_img,   p_snapshot);
    lv_img_set_angle(p_container_img, p_degree_cnt*900 + p_degree_delta*100);
}

static void heartrate_event_key_cb_x(lv_event_t *e)
{

    
//    lv_img_dsc_t* snapshot = (void*)lv_img_get_src(p_container_img);
//    if(snapshot) {
//        lv_snapshot_free(snapshot);
//    }
    if(p_snapshot == NULL) {
        p_snapshot = lv_snapshot_take(p_container_layout, LV_IMG_CF_GDX_RGB565);
        
        lv_obj_clear_flag(p_container_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_container_layout, LV_OBJ_FLAG_HIDDEN);
    }
    
    if(p_container_timer == NULL) {
        p_container_timer = lv_timer_create(rotate_timer, 20,  &user_data);
        lv_timer_pause(p_container_timer);
    }
    
    if(is_timer_running) {
        return;
    } else {
        is_timer_running = true;

        lv_img_set_src(p_container_img,   p_snapshot);
        lv_img_set_angle(p_container_img, p_degree_cnt*900 + p_degree_delta*100);

        lv_timer_reset(p_container_timer);
        lv_timer_resume(p_container_timer);
    }
    
    printf("KEY PRESS\r\n");
}



static void heartrate_event_key_cb_y(lv_event_t *e) {

    if(p_snapshot == NULL) {
        p_snapshot = lv_snapshot_take(p_container_layout, LV_IMG_CF_GDX_RGB565);
        
        lv_obj_clear_flag(p_container_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_container_layout, LV_OBJ_FLAG_HIDDEN);
    }
    
    lv_img_set_src(p_container_img,   p_snapshot);
        
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p_container_img);
    //lv_anim_set_ready_cb(&a, scroll_anim_ready_cb);

    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);  //回调执行函数,设置角度
    lv_anim_set_values(&a, 0, 900);      //0 ~ 360度
    lv_anim_set_time(&a, 1000);              //动画500ms
    lv_anim_set_delay(&a, 0);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    
    lv_anim_start(&a);
}

extern void lv_scr_load_anim_rotate(uint32_t src, uint32_t time, uint32_t delay, uint32_t start, uint32_t end, uint32_t anim);

static uint32_t s_anim = 0;
static void heartrate_event_key_cb(lv_event_t *e) {

    if(p_snapshot == NULL) {
        p_snapshot = lv_snapshot_take(p_container_layout, LV_IMG_CF_GDX_RGB565);
        
       // lv_obj_clear_flag(p_container_img, LV_OBJ_FLAG_HIDDEN);
        //lv_obj_add_flag(p_container_layout, LV_OBJ_FLAG_HIDDEN);
    }
    
    static bool is_rotate = false;
    
    if(!is_rotate) {
        lv_scr_load_anim_rotate((uint32_t)p_snapshot->data, 800, 0, 0, 1800, s_anim++%5);
        is_rotate = true;
    } else {
        lv_scr_load_anim_rotate((uint32_t)p_snapshot->data, 800, 0, 1800, 3600, s_anim++%5);
        is_rotate = false;
    }
}


static void heartrate_event_cb(lv_event_t *e)
{
    if (e->code == LV_EVENT_DELETE)
    {
        if (_hb_animation_timer)
        {
            lv_timer_del(_hb_animation_timer);
            _hb_animation_timer = NULL;
        }
    }
    else if (e->code == LV_EVENT_READY)
    {
        if (_hb_animation_timer)
        {
            // _set_heart_beat_animation(true);
        }
        else
        {
            _hb_animation_timer = lv_timer_create(_hb_timer_callback, 350, NULL);
        }
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
        if (_hb_animation_timer)
        {
            lv_timer_pause(_hb_animation_timer);
        }
    }
}


/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
lv_obj_t *lv_card_layout_heartrate_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    
    lv_obj_t *p_con_img = lv_img_create(p_window);
    lv_obj_set_size(p_con_img, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_pos(p_con_img, 0, 0);
    lv_obj_add_flag(p_con_img, LV_OBJ_FLAG_HIDDEN);
    
    p_container_img = p_con_img;


    lv_obj_t *p_con_win = lv_obj_create(p_window);
    lv_obj_set_size(p_con_win, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_pos(p_con_win, 0, 0);
    p_container_layout  = p_con_win;

    // title
    lv_obj_t *_heart_title = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_title, &lv_font_montserrat_30, LV_STATE_DEFAULT); // 30
    lv_label_set_text_static(_heart_title, "Heart Rate");
    lv_obj_set_pos(_heart_title, HR_TITLE_X, HR_TITLE_Y);

    // heart icon
    lv_obj_t *_heart_icon = lv_img_create(p_con_win);
    lv_img_set_src(_heart_icon, &wd_img_hr_icon_00001);
    lv_obj_set_pos(_heart_icon, HR_ICON_X, HR_ICON_Y);

    // heart value
    _heart_val = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_val, &lv_font_montserrat_48_gdx, LV_STATE_DEFAULT); // 60
    lv_obj_set_style_text_color(_heart_val, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_heart_val, "%d", 80);
    lv_obj_set_pos(_heart_val, HR_VAL_X, HR_VAL_Y);

    // heart unit
    lv_obj_t *_heart_unit = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT); // 30
    lv_obj_set_style_text_color(_heart_unit, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_static(_heart_unit, "BPM");
    lv_obj_set_pos(_heart_unit, HR_UNIT_X, HR_UNIT_Y);

    // measure time
    _heart_meas_time = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_meas_time, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_obj_set_style_text_color(_heart_meas_time, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_heart_meas_time, "%s", "5 Mins Ago");
    lv_obj_set_pos(_heart_meas_time, HR_MEAS_TIME_X, HR_MEAS_TIME_Y);

    // chart
    _heart_chart = lv_img_create(p_con_win);
    lv_img_set_src(_heart_chart, &wd_img_table_bg_scaled);
    lv_obj_set_pos(_heart_chart, HR_CHART_X, HR_CHART_Y);

    // heart max icon
    lv_obj_t *_heart_max_icon = lv_img_create(p_con_win);
    lv_img_set_src(_heart_max_icon, &wd_img_hr_max);
    lv_obj_set_pos(_heart_max_icon, HR_MAX_ICON_X, HR_MAX_ICON_Y);

    // heart max value
    _heart_max = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_max, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_label_set_text_fmt(_heart_max, "%d", 100);
    lv_obj_set_pos(_heart_max, HR_MAX_VAL_X, HR_MAX_VAL_Y);

    // heart min icon
    lv_obj_t *_heart_min_icon = lv_img_create(p_con_win);
    lv_img_set_src(_heart_min_icon, &wd_img_hr_min);
    lv_obj_set_pos(_heart_min_icon, HR_MIN_ICON_X, HR_MIN_ICON_Y);

    // heart min value
    _heart_min = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_min, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_label_set_text_fmt(_heart_min, "%d", 47);
    lv_obj_set_pos(_heart_min, HR_MIN_VAL_X, HR_MIN_VAL_Y);

    // heart line
    for (uint8_t i = 0; i < 24; i++)
    {
        _heart_val_line[i] = lv_line_create(p_con_win);
        lv_obj_set_style_line_rounded(_heart_val_line[i], true, 0);
        lv_obj_set_style_line_width(_heart_val_line[i], 4, 0);
        lv_obj_set_style_line_color(_heart_val_line[i], lv_color_make(255, 45, 54), 0);
        lv_obj_clear_flag(_heart_val_line[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // heart ticks
    _heart_chart_max_ticks = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_chart_max_ticks, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_heart_chart_max_ticks, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_heart_chart_max_ticks, "200");
    lv_obj_set_pos(_heart_chart_max_ticks, HR_CHART_Y_MAX_TICKS_X, HR_CHART_Y_MAX_TICKS_Y);

    _heart_chart_min_ticks = lv_label_create(p_con_win);
    lv_obj_set_style_text_font(_heart_chart_min_ticks, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_heart_chart_min_ticks, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_heart_chart_min_ticks, "0");
    lv_obj_set_pos(_heart_chart_min_ticks, HR_CHART_Y_MIN_TICKS_X, HR_CHART_Y_MIN_TICKS_Y);

    _hr_set_max_min_val(hr_max_data, hr_min_data);

    //lv_obj_add_event_cb(p_window, heartrate_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(p_window, heartrate_event_key_cb, LV_EVENT_KEY, NULL);
    
    printf("HR Window OBJ: 0x%08x \r\n", (uint32_t)p_window);

    return p_window;
}
