/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_font.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"
#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "disp_driver.h"
#include "lv_port_disp.h"
#include "lv_wms.h"
#include "lv_clock_hands_draw.h"
/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define LV_IMG_PSRAM_CACHE_ENABLE  1

/*
 * GLOBAL VARIABLE DECLERATIONS
 *****************************************************************************************
 */
LV_IMG_DECLARE(flower);

/*
 * GLOBAL FUNCTION DECLERATIONS
 *****************************************************************************************
 */
void lv_port_res_mode_set(uint8_t mode);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
LV_IMG_DECLARE(lv_img_watch3_bg);
LV_IMG_DECLARE(lv_img_watch3_hour1);
LV_IMG_DECLARE(lv_img_watch3_min1);
LV_IMG_DECLARE(lv_img_watch3_sec1);
LV_IMG_DECLARE(flower);
LV_IMG_DECLARE(center);
LV_IMG_DECLARE(hour);
LV_IMG_DECLARE(minute);
LV_IMG_DECLARE(second);

static lv_obj_t* img_hour;
static lv_obj_t* img_second;
static lv_obj_t* img_minute;
static lv_obj_t* flower_obj;
lv_obj_t* g_flower_clock_view = NULL;

#if LV_IMG_PSRAM_CACHE_ENABLE
static lv_img_dsc_t s_wd_img_background;
static void* s_background_data = NULL;
static lv_img_dsc_t s_wd_img_live_wallpaer_flower;
static void* s_flower_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_second;
static void* s_second_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_minute;
static void* s_minute_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_hour;
static void* s_hour_data = NULL;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void gx_flower_clock_image_mem_free(void){
#if LV_IMG_PSRAM_CACHE_ENABLE
    if( NULL != s_background_data)
    {
        app_graphics_mem_free(s_background_data);
        s_background_data = NULL;
    }

    if( NULL != s_flower_data)
    {
        app_graphics_mem_free(s_flower_data);
        s_flower_data = NULL;
    }

    if( NULL != s_second_data)
    {
        app_graphics_mem_free(s_second_data);
        s_second_data = NULL;
    }

    if( NULL != s_minute_data)
    {
        app_graphics_mem_free(s_minute_data);
        s_minute_data = NULL;
    }

    if( NULL == s_hour_data)
    {
        app_graphics_mem_free(s_hour_data);
        s_hour_data = NULL;
    }
#endif
}

static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    gx_flower_clock_image_mem_free();
    lv_wms_go_to_window(lv_clock_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_flower_clock_layout_destroy(void){
    if(NULL == g_flower_clock_view) return;
    lv_clk_hand_stop_run();
    lv_wms_deinit(g_flower_clock_view);
    lv_obj_del(g_flower_clock_view);
    g_flower_clock_view = NULL;
}

lv_obj_t* lv_flower_clock_layout_create(void){
    if(NULL != g_flower_clock_view) return g_flower_clock_view;
    g_flower_clock_view = lv_obj_create(NULL);
    lv_obj_t* obj = g_flower_clock_view;

    lv_obj_t *img_background = lv_img_create(obj);
    /***save the flower in PSRAM for faster speed***************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    memcpy(&s_wd_img_background, &wd_img_live_flower_watchface1, sizeof(lv_img_dsc_t));
    if( NULL == s_background_data)
    {
        s_background_data = app_graphics_mem_malloc(wd_img_live_flower_watchface1.data_size);
    }
    memcpy(s_background_data, wd_img_live_flower_watchface1.data, wd_img_live_flower_watchface1.data_size);
    s_wd_img_background.data = s_background_data;
    lv_img_set_src(img_background, &s_wd_img_background);
    #else
    lv_img_set_src(img_background, &wd_img_live_flower_watchface1);
    #endif
    /**********************************************************************************************/
    // Notice the background image is RGB565 format
    lv_obj_set_pos(img_background, 0, 0);

    flower_obj = lv_img_create(obj);
    /***save the flower in PSRAM for faster speed***************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    lv_port_res_mode_set(2);
    memcpy(&s_wd_img_live_wallpaer_flower, &wd_img_live_wallpaer_flower, sizeof(lv_img_dsc_t));
    if( NULL == s_flower_data)
    {
        s_flower_data = app_graphics_mem_malloc(wd_img_live_wallpaer_flower.data_size);
    }
    memcpy(s_flower_data, wd_img_live_wallpaer_flower.data, wd_img_live_wallpaer_flower.data_size);
    s_wd_img_live_wallpaer_flower.data = s_flower_data;
    lv_img_set_src(flower_obj, &s_wd_img_live_wallpaer_flower);
    #else
    lv_img_set_src(flower_obj, &wd_img_live_wallpaer_flower);
    #endif
    /**********************************************************************************************/
    lv_obj_set_pos(flower_obj, 96, 96);
    lv_img_set_pivot(flower_obj, 130, 130);

    img_second = lv_img_create(obj);
    /***save the second in PSRAM for faster speed***************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    memcpy(&s_wd_img_live_flower_second, &wd_img_live_flower_second, sizeof(lv_img_dsc_t));
    if( NULL == s_second_data)
    {
        s_second_data = app_graphics_mem_malloc(wd_img_live_flower_second.data_size);
    }
    memcpy(s_second_data, wd_img_live_flower_second.data, wd_img_live_flower_second.data_size);
    s_wd_img_live_flower_second.data = s_second_data;
    lv_img_set_src(img_second, &s_wd_img_live_flower_second);
    #else
    lv_img_set_src(img_second, &wd_img_live_flower_second);
    #endif
    /**********************************************************************************************/
    lv_obj_set_pos(img_second, 96 + 130, 96 + 130 - 2);
    lv_img_set_pivot(img_second, 0, 2);

    img_minute = lv_img_create(obj);
    /***save the minute in PSRAM for faster speed***************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    memcpy(&s_wd_img_live_flower_minute, &wd_img_live_flower_minute, sizeof(lv_img_dsc_t));
    if( NULL == s_minute_data)
    {
        s_minute_data = app_graphics_mem_malloc(wd_img_live_flower_minute.data_size);
    }
    memcpy(s_minute_data, wd_img_live_flower_minute.data, wd_img_live_flower_minute.data_size);
    s_wd_img_live_flower_minute.data = s_minute_data;
    lv_img_set_src(img_minute, &s_wd_img_live_flower_minute);
    #else
    lv_img_set_src(img_minute, &wd_img_live_flower_minute);
    #endif
    /**********************************************************************************************/
    lv_obj_set_pos(img_minute, 96 + 130, 96 + 130 - 8);
    lv_img_set_pivot(img_minute, 0, 8);

    img_hour = lv_img_create(obj);
    /***save the hour in PSRAM for faster speed****************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    memcpy(&s_wd_img_live_flower_hour, &wd_img_live_flower_hour, sizeof(lv_img_dsc_t));
    if( NULL == s_hour_data)
    {
        s_hour_data = app_graphics_mem_malloc(wd_img_live_flower_hour.data_size);
    }
    memcpy(s_hour_data, wd_img_live_flower_hour.data, wd_img_live_flower_hour.data_size);
    s_wd_img_live_flower_hour.data = s_hour_data;
    lv_img_set_src(img_hour, &s_wd_img_live_flower_hour);
    lv_port_res_mode_set(1);
    #else
    lv_img_set_src(img_hour, &wd_img_live_flower_hour);
    lv_port_res_mode_set(1);
    #endif
    /**********************************************************************************************/
    lv_obj_set_pos(img_hour, 96 + 130, 96 + 130 - 10);
    lv_img_set_pivot(img_hour, 0, 10);
    
    lv_obj_t *img_center = lv_img_create(obj);
    lv_img_set_src(img_center, &wd_img_live_flower_center);
    lv_obj_set_pos(img_center, 96 + 130 - 10, 96 + 130 - 10);

    lv_obj_update_layout(obj);
    lv_obj_add_event_cb(g_flower_clock_view, gx_home_layout_long_press_handler, LV_EVENT_LONG_PRESSED, NULL);
    lv_clk_set_hour_hand(img_hour);
    lv_clk_set_min_hand(img_minute);
    lv_clk_set_sec_hand(img_second);
    lv_clk_set_bg_cb(flower_obj, NULL);
    lv_clk_hand_start_run();

    lv_wms_init(g_flower_clock_view);
    lv_wms_self_destroy_func_set(g_flower_clock_view, lv_flower_clock_layout_destroy);
    g_cur_clock_view_create_func = lv_flower_clock_layout_create;
    lv_wms_left_create_func_set(g_flower_clock_view, g_cur_list_view_create_func);
    lv_wms_right_create_func_set(g_flower_clock_view, lv_activity_layout_create);
    lv_wms_top_create_func_set(g_flower_clock_view, lv_status_bar_layout_create);
    lv_wms_bottom_create_func_set(g_flower_clock_view, lv_message_layout_create);

    lv_scr_load(g_flower_clock_view);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_flower_clock_view;
}
