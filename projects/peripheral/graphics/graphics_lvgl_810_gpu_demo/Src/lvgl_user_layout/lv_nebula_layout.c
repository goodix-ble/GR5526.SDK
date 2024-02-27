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
#include "lv_nebula.h"

/**********************
 *      TYPEDEFS
 **********************/
typedef enum {
    ICON_1_1_SETTINGS = 0, 
    ICON_1_2_FACE_GALLERY, 
    ICON_1_3_HEART_RATE, 
    ICON_1_4_DAILY_ACTIVITY, 
    ICON_1_5_DRAW_CUBE, 
    ICON_1_6_DRAW_ARCS, 

    ICON_1_7_JUMPING,
    ICON_1_8_DRAW_CIRCLE,
    ICON_1_9_MASK_STENCIL,
    ICON_1_10_WORKOUTS,
    ICON_1_37_COMPRESS,
    ICON_1_12_STRESS,

    ICON_1_15_ALIPAY, 
    ICON_1_16_ACTIVITY_RECORDS, 
    ICON_1_17_MUSIC,
    ICON_1_18_WECHAT,
    ICON_1_13_SPO2,
    ICON_1_14_SLEEP,

    ICON_1_21_FLASHLIGHT,
    ICON_1_22_EVENTS, 
    ICON_1_19_BREATHING,
    ICON_1_20_WEATHER,
    ICON_1_23_CALENDAR,
    ICON_1_24_PHONE ,

    ICON_1_25_HIMALAYA,
    ICON_1_26_FIND_PHONE,
    ICON_1_27_WORLD_CLOCK, 
    ICON_1_28_WALLET, 
    ICON_1_29_APP_MARKET,
    ICON_1_30_COUNTER, 

    ICON_1_33_STOP_WATCH, 
    ICON_1_34_RECORDER,
    ICON_1_35_PHOTO, 
    ICON_1_36_TIMER,
    ICON_1_31_NETEASE_MUSIC, 
    ICON_1_32_ALARM, 
}lv_nebula_icon_t;

const uint32_t g_app_png_icon_addrs[] = 
{
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
    ADDR_1_37_COMPRESS,
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

/*
 * GLOBAL VARIALBLE DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t* g_nebula_view = NULL;

static bool gw_check_view(lv_point_t *act_point, lv_area_t *click_area)
{
    if( act_point->x  >= (click_area->x1) &&\
        act_point->x  <=(click_area->x2) &&\
        act_point->y  >= (click_area->y1) &&\
        act_point->y  <=(click_area->y2)){
        return true;
    }
    return false;
}

static bool click_event_cb(lv_event_t * e)
{
    lv_nebula_t* obj = (lv_nebula_t*)e->current_target;
    lv_indev_t * indev_act = lv_indev_get_act();
    lv_point_t click_point = {0, 0};
    lv_indev_get_point(indev_act, &click_point);
    uint8_t index = 0xFF;
    lv_area_t click_area;
    uint8_t i = 0;
    for( i = 0; i < CFG_ICON_CNT; i++){
        if(0 == obj->size_array[i]) continue;
        click_area.x1 = obj->x_array[i];
        click_area.x2 = obj->x_array[i] + obj->size_array[i];
        click_area.y1 = obj->y_array[i];
        click_area.y2 = obj->y_array[i] + obj->size_array[i];
        if(gw_check_view(&click_point, &click_area))
        {
            index = i;
            break;
        }
    }
    // printf("index: %d\n", index);
    if( i != index) return false; // find nothing
    switch((index % 36)){
        case ICON_1_1_SETTINGS:{
            lv_wms_go_to_window(lv_setting_view_create);
            break;
        }
        default:{
            break;
        }
    }
    return true;
}

static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_list_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}

static void scroll_x_anim(void * anim_obj, int32_t value)
{
    lv_nebula_t* obj = (lv_nebula_t*)anim_obj;
    ((lv_nebula_t *)obj)->offset.x = value;
    // Limit the offset within normal range
    // switch inside and outside scroll when reaching boundaries
    if(obj->offset.x < MIN_OFFSET){
        obj->offset.x = MIN_OFFSET;
        lv_wms_right_create_func_set(g_nebula_view, g_cur_clock_view_create_func);
    }else{
        lv_wms_right_create_func_set(g_nebula_view, NULL);
    }
    if(obj->offset.x > MAX_OFFSET){
        obj->offset.x = MAX_OFFSET;
        lv_wms_left_create_func_set(g_nebula_view, lv_heartrate_layout_create);
    }else{
        lv_wms_left_create_func_set(g_nebula_view, NULL);
    }
    if(g_nebula_view) lv_obj_invalidate((lv_obj_t*)g_nebula_view);
}

static void scroll_y_anim(void * anim_obj, int32_t value)
{
    lv_nebula_t* obj = (lv_nebula_t*)anim_obj;
    ((lv_nebula_t *)obj)->offset.y = value;
    // Limit the offset within normal range
    if (obj->offset.y < MIN_OFFSET){
       obj->offset.y = MIN_OFFSET;
    }
    if( obj->offset.y > MAX_OFFSET){
       obj->offset.y = MAX_OFFSET;
    }
    if(g_nebula_view) lv_obj_invalidate((lv_obj_t*)g_nebula_view);
}

int16_t gw_nebual_end_offset_calculate(int16_t offset)
{
    int16_t align_offset = offset;
    int16_t remain = align_offset % ICON_SPACE_SIZE;
    if(LV_ABS(remain) >= ICON_ALIGN_SIZE){
        if(remain > 0){
            align_offset -= remain;
            align_offset += ICON_SPACE_SIZE;
        }else{
            align_offset -= remain;
            align_offset += -ICON_SPACE_SIZE;
        }
    }else{
        align_offset -= remain;
    }
    /* Select the four center style by add align size */
    return align_offset + ICON_ALIGN_SIZE;
}

static void scroll_anim_start(lv_dir_t dir, lv_coord_t offset){
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, g_nebula_view);
    lv_nebula_t* obj = (lv_nebula_t*)g_nebula_view;
    if(LV_DIR_HOR == dir){
        lv_anim_set_values(&a, obj->offset.x, gw_nebual_end_offset_calculate(obj->offset.x + offset));
        lv_anim_set_exec_cb(&a, scroll_x_anim);
    }else{
        lv_anim_set_values(&a, obj->offset.y, gw_nebual_end_offset_calculate(obj->offset.y + offset));
        lv_anim_set_exec_cb(&a, scroll_y_anim);
    }
    // lv_anim_set_time(&a, 300); // default is 500
    lv_anim_set_path_cb(&a, lv_anim_path_linear);
    lv_anim_start(&a);
}

static void scroll_event_cb(lv_event_t * e)
{
    if(lv_wms_is_in_busy_state()){
        return;
    }
    lv_indev_t * indev_act = lv_indev_get_act();
    lv_nebula_t* obj = (lv_nebula_t*)e->current_target;
    obj->offset.x += indev_act->proc.types.pointer.vect.x;
    obj->offset.y += indev_act->proc.types.pointer.vect.y;
    // Limit the offset within normal range
    // switch inside and outside scroll when reaching boundaries
    if(obj->offset.x < MIN_OFFSET){
        obj->offset.x = MIN_OFFSET;
        lv_wms_right_create_func_set(g_nebula_view, g_cur_clock_view_create_func);
    }else{
        lv_wms_right_create_func_set(g_nebula_view, NULL);
    }
    if(obj->offset.x > MAX_OFFSET){
        obj->offset.x = MAX_OFFSET;
        lv_wms_left_create_func_set(g_nebula_view, lv_heartrate_layout_create);
    }else{
        lv_wms_left_create_func_set(g_nebula_view, NULL);
    }

    // Limit the offset within normal range
    if (obj->offset.y < MIN_OFFSET) obj->offset.y = MIN_OFFSET;
    if( obj->offset.y > MAX_OFFSET) obj->offset.y = MAX_OFFSET;

    static lv_point_t _scroll_abs_sum = {0, 0};
    static lv_point_t _scroll_throw_sum = {0, 0};
    static uint32_t _x_tick_pressed = 0;
    switch(e->code){
        case LV_EVENT_PRESSED:{
            _x_tick_pressed = xTaskGetTickCount();
            _scroll_abs_sum.x = 0;
            _scroll_abs_sum.y = 0;
            _scroll_throw_sum.x = 0;
            _scroll_throw_sum.y = 0;
            break;
        }
        case LV_EVENT_PRESSING:{
            _scroll_abs_sum.x += LV_ABS(indev_act->proc.types.pointer.vect.x);
            _scroll_abs_sum.y += LV_ABS(indev_act->proc.types.pointer.vect.y);
            /*Calculate the vector and apply a low pass filter: new_value = 0.5 * old_value + 0.5 * new_offset*/
            _scroll_throw_sum.x = (_scroll_throw_sum.x * 4) >> 3;
            _scroll_throw_sum.x += ((indev_act->proc.types.pointer.vect.x * 4) >> 3);
            _scroll_throw_sum.y = (_scroll_throw_sum.y * 4) >> 3;
            _scroll_throw_sum.y += ((indev_act->proc.types.pointer.vect.y * 4) >> 3);
            // Implement the long pressing event
            uint32_t tick_diff = xTaskGetTickCount() - _x_tick_pressed;
            if((_scroll_abs_sum.x < 10) && (_scroll_abs_sum.y < 10)){
                if(tick_diff > LV_INDEV_DEF_LONG_PRESS_TIME) {
                    gx_home_layout_long_press_handler(e);
                    return;
                }
            }
            break;
        }
        case LV_EVENT_RELEASED:{
            uint32_t tick_diff = xTaskGetTickCount() - _x_tick_pressed;
            if((_scroll_abs_sum.x < 10) && (_scroll_abs_sum.y < 10)){
                if(tick_diff < LV_INDEV_DEF_LONG_PRESS_TIME) {
                    if(click_event_cb(e)) return;
                }
            }
            // start the animation effect to align the icons
            scroll_anim_start(LV_DIR_HOR, 15*_scroll_throw_sum.x);
            scroll_anim_start(LV_DIR_VER, 15*_scroll_throw_sum.y);
            break;
        }
        default:{
            break;
        }
    }
    // printf("%d,%d\n", obj->offset.x, obj->offset.y);
    if(g_nebula_view) lv_obj_invalidate((lv_obj_t*)g_nebula_view);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_nebula_layout_destroy(void){
    if(NULL == g_nebula_view) return;
    lv_wms_deinit(g_nebula_view);
    lv_obj_del(g_nebula_view);
    g_nebula_view = NULL;
}

lv_obj_t* lv_nebula_layout_create(void){
    if(NULL != g_nebula_view) return g_nebula_view;
    g_nebula_view = lv_nebula_create(NULL);
    lv_obj_clear_flag(g_nebula_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_nebula_view, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(g_nebula_view, 454, 454);
    lv_obj_add_flag(g_nebula_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(g_nebula_view, LV_DIR_NONE);
    lv_obj_add_event_cb(g_nebula_view, scroll_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(g_nebula_view, scroll_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(g_nebula_view, scroll_event_cb, LV_EVENT_RELEASED, NULL);

    lv_wms_init(g_nebula_view);
    lv_wms_self_destroy_func_set(g_nebula_view, lv_nebula_layout_destroy);
    lv_wms_left_create_func_set(g_nebula_view, lv_heartrate_layout_create);
    lv_wms_right_create_func_set(g_nebula_view, g_cur_clock_view_create_func);
    g_cur_list_view_create_func = lv_nebula_layout_create;
    lvgl_mem_used_dump(__func__, __LINE__);

    lv_scr_load(g_nebula_view);
    return g_nebula_view;
}
