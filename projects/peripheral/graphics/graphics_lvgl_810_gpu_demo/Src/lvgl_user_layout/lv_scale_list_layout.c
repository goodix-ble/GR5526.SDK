/**
 * @file lv_nebula.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "grx_hal.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "lv_port_disp.h"
#include "hal_gfx_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_graphics_gpu.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"
#include "disp_driver.h"
#include "hal_gfx_programHW.h"
#include "hal_gfx_regs.h"
#include "hal_gfx_ringbuffer.h"
#include "hal_gfx_rasterizer.h"
#include "hal_gfx_raster.h"
#include "lv_scale_list.h"
#include "lv_layout_manager.h"
/*********************
 *      DEFINES
 *********************/
#define ICON_NUM    30
#define ELASTIC_SLOWNESS_FACTOR 4   /*Scrolling on elastic parts are slower by this factor*/
#define SCROLL_ANIM_TIME_MIN    30    /*ms*/
#define SCROLL_ANIM_TIME_MAX    600    /*ms*/
#define SCROLL_LIMIT            15
/**********************
 *      TYPEDEFS
 **********************/
 
/**********************
 *      DECLARATIONS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/


/**********************
 *  STATIC VARIABLES
 **********************/


/**********************
 *      MACROS
 **********************/

/**********************
 *      STATIC VARIABLES
 **********************/

const uint32_t app_png_icon_addrs[] = 
{
    ADDR_2_1_SETTINGS, 
    ADDR_2_2_FACE_GALLERY, 
    ADDR_2_3_HEART_RATE, 
    ADDR_2_4_DAILY_ACTIVITY, 
    ADDR_2_5_DRAW_CUBE, 
    ADDR_2_6_DRAW_ARCS, 

    ADDR_2_7_JUMPING,
    ADDR_2_8_DRAW_CIRCLE,
    ADDR_2_9_MASK_STENCIL,
    ADDR_1_37_COMPRESS,
    ADDR_2_11_MESSAGES,
    ADDR_2_12_STRESS,

    ADDR_2_15_ALIPAY, 
    ADDR_2_16_ACTIVITY_RECORDS, 
    ADDR_2_17_MUSIC,
    ADDR_2_18_WECHAT,
    ADDR_2_13_SPO2,
    ADDR_2_14_SLEEP,

    ADDR_2_21_FLASHLIGHT,
    ADDR_2_22_EVENTS, 
    ADDR_2_19_BREATHING,
    ADDR_2_20_WEATHER,
    ADDR_2_23_CALENDAR,
    ADDR_2_24_PHONE ,

    ADDR_2_25_HIMALAYA,
    ADDR_2_26_FIND_PHONE,
    ADDR_2_27_WORLD_CLOCK, 
    ADDR_2_28_WALLET, 
    ADDR_2_29_APP_MARKET,
    ADDR_2_30_COUNTER, 

    ADDR_2_33_STOP_WATCH, 
    ADDR_2_34_RECORDER,
    ADDR_2_35_PHOTO, 
    ADDR_2_36_TIMER,
    ADDR_2_31_NETEASE_MUSIC, 
    ADDR_2_32_ALARM, 
    
/*--------------------------------------------*/
    ADDR_1_1_SETTINGS, 
    ADDR_1_2_FACE_GALLERY, 
    ADDR_1_3_HEART_RATE, 
    ADDR_1_4_DAILY_ACTIVITY, 
    ADDR_1_5_DRAW_CUBE, 
    ADDR_1_6_DRAW_ARCS, 

    ADDR_1_7_JUMPING,
    ADDR_1_8_DRAW_CIRCLE,
    ADDR_1_9_MASK_STENCIL,
    ADDR_1_10_WORKOUTS,
    ADDR_1_11_MESSAGES,
    ADDR_1_12_STRESS,

    ADDR_1_15_ALIPAY, 
    ADDR_1_16_ACTIVITY_RECORDS, 
    ADDR_1_17_MUSIC,
    ADDR_1_18_WECHAT,
    ADDR_1_13_SPO2,
    ADDR_1_14_SLEEP,

    ADDR_1_21_FLASHLIGHT,
    ADDR_1_22_EVENTS, 
    ADDR_1_19_BREATHING,
    ADDR_1_20_WEATHER,
    ADDR_1_23_CALENDAR,
    ADDR_1_24_PHONE ,

    ADDR_1_25_HIMALAYA,
    ADDR_1_26_FIND_PHONE,
    ADDR_1_27_WORLD_CLOCK, 
    ADDR_1_28_WALLET, 
    ADDR_1_29_APP_MARKET,
    ADDR_1_30_COUNTER, 

    ADDR_1_33_STOP_WATCH, 
    ADDR_1_34_RECORDER,
    ADDR_1_35_PHOTO, 
    ADDR_1_36_TIMER,
    ADDR_1_31_NETEASE_MUSIC, 
    ADDR_1_32_ALARM, 
};

static const char str_infos[][15]= 
{
    "设置", 
    "表盘商店",
    "心率",
    "每日活动",
    "画立方体",
    "画弧线", 
    
    "跳跃动画",
    "画圆",
    "蒙板",
    "TSC4",
    "压力",
    "信息",

    "支付宝", 
    "运动记录",
    "音乐",
    "微信",
    "血氧",
    "睡眠",

    "手电筒",
    "日程安排",
    "呼吸训练",
    "天气",
    "日历",
    "电话",

    "喜马拉雅",
    "找手机",
    "世界时钟",
    "钱包",
    "应用商店",
    "计算器",

    "秒表", 
    "录音", 
    "拍照", 
    "计时器",
    "网易音乐",
    "闹钟", 
};
 /*
 * GLOBAL VARIALBLE DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t* g_scale_list_view = NULL;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_list_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}

static void scroll_anim_ready_cb(lv_anim_t * a)
{

}

static void scroll_x_anim(void * obj, int32_t v)
{
    if(!g_scale_list_view)  return;
    ((lv_scale_list_t *)obj)->attribute.slide_offset = v;
    lv_obj_invalidate((lv_obj_t*)g_scale_list_view);
}

/*------------------------------------------------------*/
#define SAVE_PRESSING_COUNT  3
static int16_t anim_dy;
static int16_t s_pressing_point[SAVE_PRESSING_COUNT];


static void anim_save_pressing_point(lv_point_t *act_point)
{
    uint8_t i = 0;
    for(i=0; i<SAVE_PRESSING_COUNT-1; i++)
    {
        s_pressing_point[i] = s_pressing_point[i+1];
    }
    s_pressing_point[SAVE_PRESSING_COUNT-1] = act_point->y;
}

static void anim_released_cal_anim_dy(void)
{
    for(uint8_t i=0; i<SAVE_PRESSING_COUNT; i++)
    {
        if(s_pressing_point[i] != 0)
        {
            anim_dy = s_pressing_point[SAVE_PRESSING_COUNT-1] - s_pressing_point[i];
            break;
        }
    }
    memset(s_pressing_point, 0, sizeof(int16_t) * SAVE_PRESSING_COUNT);
}

static void scroll_end_to_align(lv_scale_list_t* obj)
{
    int16_t anim_offset = 0;
    int16_t MIN_Y_OFFSET = lv_scale_list_get_max_offset((lv_obj_t* )obj);
    int16_t MAX_Y_OFFSET = 0;
    int16_t scale_ratio = 0;
    
    scale_list_type_t type;
    lv_scale_list_get_mode((lv_obj_t* )obj, &type);

    if (type == SCALE_NORMAL_MODE){
        scale_ratio = 8;
    }else if (type == SCALE_CIRCLE_MODE){
        scale_ratio = 6;
    }else{
        scale_ratio = 4;
    }

    if (obj->attribute.slide_offset < MIN_Y_OFFSET){
        anim_offset = MIN_Y_OFFSET - obj->attribute.slide_offset;
    }else if (obj->attribute.slide_offset > MAX_Y_OFFSET){
        anim_offset = MAX_Y_OFFSET - obj->attribute.slide_offset;
    }else{
        if (LV_ABS(anim_dy) < 10)
        {
            return;
        }
        anim_offset = anim_dy * scale_ratio;
        if (obj->attribute.slide_offset + anim_offset < MIN_Y_OFFSET){
            anim_offset = MIN_Y_OFFSET - obj->attribute.slide_offset;
        }
        if (obj->attribute.slide_offset + anim_offset > MAX_Y_OFFSET){
            anim_offset = MAX_Y_OFFSET - obj->attribute.slide_offset;
        }
    }
    uint32_t t = lv_anim_speed_to_time((lv_disp_get_hor_res(NULL)/5 ), 0, anim_offset);
    if(t < SCROLL_ANIM_TIME_MIN) t = SCROLL_ANIM_TIME_MIN;
    if(t > SCROLL_ANIM_TIME_MAX) t = SCROLL_ANIM_TIME_MAX;
    
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_ready_cb(&a, scroll_anim_ready_cb);
    lv_anim_set_values(&a, obj->attribute.slide_offset, obj->attribute.slide_offset + anim_offset);
    lv_anim_set_exec_cb(&a, scroll_x_anim);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}
static void lv_reset_wms_create(lv_obj_t* layout)
{
    lv_wms_left_create_func_set(layout, g_cur_list_view_create_func);
    lv_wms_right_create_func_set(layout, NULL);
}

static void click_cb(uint8_t index)
{
    //printf("click item = %s\r\n", str_infos[index]);
    switch(index)
    {
        case 0:
            lv_wms_go_to_window(lv_setting_view_create);
            break;
        case 1:
            lv_wms_go_to_window(lv_clock_view_select_create);
            break;
        case 2:
            lv_wms_go_to_window(lv_heartrate_layout_create);
            lv_reset_wms_create(lv_heartrate_layout_create());
            break;
        case 3:
            lv_wms_go_to_window(lv_activity_layout_create);
            lv_reset_wms_create(lv_activity_layout_create());
            break;
        case 4:
            lv_wms_go_to_window(lv_graphics_cube_layout_create);
            break;
        case 5:
            lv_wms_go_to_window(lv_graphics_arc_layout_create);
            break;
        case 6:
            lv_wms_go_to_window(lv_graphics_jump_layout_create);
            break;
        case 7:
            lv_wms_go_to_window(lv_graphics_circle_layout_create);
            break;
        case 8:
            lv_wms_go_to_window(lv_graphics_stencil_layout_create);
            break;
        case 9:
            lv_wms_go_to_window(lv_graphics_tsc4_layout_create);
            break;
        case 10:
            lv_wms_go_to_window(lv_volume_adjust_layout_create);
            break;
        case 11:
            lv_wms_go_to_window(lv_chart_layout_create);
            break;
        case 13:
            lv_wms_go_to_window(lv_widgets_test_view_create);
            break;
        default:
            break;
    }
}

static void scroll_event_cb(lv_event_t * e)
{
    if(lv_wms_is_in_busy_state()){
        return;
    }
    lv_indev_t * indev_act = lv_indev_get_act();
    lv_scale_list_t* obj = (lv_scale_list_t*)e->current_target;
    int16_t diff = indev_act->proc.types.pointer.vect.y;
    int16_t MIN_Y_OFFSET = lv_scale_list_get_max_offset((lv_obj_t* )obj);
    int16_t MAX_Y_OFFSET = 0;
    static lv_point_t _scroll_sum = {0, 0};
    static uint32_t _x_tick_pressed = 0;
    static uint32_t _x_tick_released = 0;
    static bool scroll_start = false;
    switch(e->code){
        case LV_EVENT_PRESSED:{
            _x_tick_pressed = xTaskGetTickCount();
            scroll_start = false;
            _scroll_sum.y = 0;
            break;
        }
        case LV_EVENT_PRESSING:{
            anim_save_pressing_point(&indev_act->proc.types.pointer.act_point);
            _scroll_sum.y += diff;
            // Update the y offset when possible
            if (obj->attribute.slide_offset < MIN_Y_OFFSET   \
                || obj->attribute.slide_offset > MAX_Y_OFFSET){
               if(diff < 0) diff -= ELASTIC_SLOWNESS_FACTOR / 2;
               if(diff > 0) diff += ELASTIC_SLOWNESS_FACTOR / 2;
               obj->attribute.slide_offset += diff / ELASTIC_SLOWNESS_FACTOR;
            }else{
               obj->attribute.slide_offset += diff;
            }
            if (!scroll_start){
                if (abs(_scroll_sum.y) > SCROLL_LIMIT){
                    scroll_start = true;
                }else{
                    _x_tick_released = xTaskGetTickCount();
                    uint32_t tick_diff = _x_tick_released - _x_tick_pressed;
                    if(tick_diff > 500) {
                        gx_home_layout_long_press_handler(e);
                    }
                }
            }
            if (scroll_start){
                lv_scale_list_layout_update((lv_obj_t *)obj);
                if(g_scale_list_view) lv_obj_invalidate((lv_obj_t*)g_scale_list_view);
            }
            break;
        }
        case LV_EVENT_RELEASED:{
            if(!scroll_start){
                lv_point_t click_point;
                uint8_t item_index;
                lv_indev_get_point(indev_act, &click_point);
                if (lv_scale_list_search_view((lv_obj_t *)obj, click_point, &item_index))
                {
                    click_cb(item_index);
                }
            }else{
                anim_released_cal_anim_dy();
                scroll_end_to_align(obj);
            }
            _scroll_sum.y = 0;
            scroll_start = false;
            break;
        }
        default:{
            break;
        }
    }
}

void lv_scale_list_layout_destroy(void){
    if(NULL == g_scale_list_view) return;
    lv_wms_deinit(g_scale_list_view);
    lv_obj_del(g_scale_list_view);
    g_scale_list_view = NULL;
}

lv_obj_t* lv_scale_list_layout_get(void)
{
    return g_scale_list_view;
}

lv_obj_t* lv_scale_list_layout_create(void){
    if(NULL != g_scale_list_view) return g_scale_list_view;
    g_scale_list_view = lv_scale_list_create(NULL);
    lv_obj_clear_flag(g_scale_list_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_scale_list_view, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(g_scale_list_view, 454, 454);
    lv_obj_add_flag(g_scale_list_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(g_scale_list_view, LV_DIR_NONE);
    lv_scale_list_add_item(g_scale_list_view, (uint32_t *)app_png_icon_addrs, (char *)str_infos, ICON_NUM);
    //lv_scr_load(g_scale_list_view);
    lv_obj_add_event_cb(g_scale_list_view, scroll_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(g_scale_list_view, scroll_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(g_scale_list_view, scroll_event_cb, LV_EVENT_RELEASED, NULL);

    lv_wms_init(g_scale_list_view);
    lv_wms_self_destroy_func_set(g_scale_list_view, lv_scale_list_layout_destroy);
    g_cur_list_view_create_func = lv_scale_list_layout_create;
    lv_wms_left_create_func_set(g_scale_list_view, lv_heartrate_layout_create);
    lv_wms_right_create_func_set(g_scale_list_view, g_cur_clock_view_create_func);
    lvgl_mem_used_dump(__func__, __LINE__);

    lv_scr_load(g_scale_list_view);
    return g_scale_list_view;
}
