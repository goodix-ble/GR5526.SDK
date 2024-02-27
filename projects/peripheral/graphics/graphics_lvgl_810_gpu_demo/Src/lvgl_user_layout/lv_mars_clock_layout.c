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
#include "lv_clock_hands_draw.h"
/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_simsun_28;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static lv_timer_t* layout_timer = NULL;
static lv_obj_t* img_bg;

static const lv_img_dsc_t* img_array[] =
{
    &wd_img_live_wallpaper_mars_1,
    &wd_img_live_wallpaper_mars_2,
    &wd_img_live_wallpaper_mars_3,
    &wd_img_live_wallpaper_mars_4,
    &wd_img_live_wallpaper_mars_5,
    &wd_img_live_wallpaper_mars_6,
    &wd_img_live_wallpaper_mars_7,
    &wd_img_live_wallpaper_mars_8,
};

static void lv_layout_timer(lv_timer_t * tmr)
{
    static uint32_t i = 0;
    i++;
    lv_img_set_src(img_bg, img_array[i%8]);  
}

static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_clock_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}

lv_obj_t* g_mars_clock_view = NULL;

void lv_mars_clock_layout_destroy(void)
{
    if(NULL == g_mars_clock_view) return;
    lv_timer_del(layout_timer);
    layout_timer = NULL;
    lv_wms_deinit(g_mars_clock_view);
    lv_obj_del(g_mars_clock_view);
    g_mars_clock_view = NULL;
}

lv_obj_t* lv_mars_clock_layout_create(void)
{
    if(NULL != g_mars_clock_view) return g_mars_clock_view;
    g_mars_clock_view = lv_obj_create(NULL);
    lv_obj_t* obj = g_mars_clock_view;

    lv_obj_t* img_cloud = lv_img_create(obj);
    lv_img_set_src(img_cloud, &wd_img_sun2);
    lv_obj_set_pos(img_cloud, 100, 145);
    
    img_bg = lv_img_create(obj);
    lv_img_set_src(img_bg, &wd_img_live_wallpaper_mars_1);
    lv_obj_set_pos(img_bg, 100, 100);
    lv_obj_set_size(img_bg, 350, 350);
    lv_img_set_size_mode(img_bg, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_zoom(img_bg, 512);
    lv_obj_t* label = lv_label_create(obj);
    lv_obj_set_style_text_font(label, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label, "20°C");
    lv_obj_set_pos(label, 143, 143);

    lv_obj_t* label3 = lv_label_create(obj);
    lv_obj_set_style_text_font(label3, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label3, "04-08");
    lv_obj_set_pos(label3, 270, 143);

    lv_obj_t* label4 = lv_label_create(obj);
    lv_obj_set_style_text_font(label4, &lv_font_montserrat_40, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label4, lv_color_white(), 0);
    lv_label_set_text(label4, "12:32");
    lv_obj_set_pos(label4, 135, 200);

    if(NULL == layout_timer){
        layout_timer = lv_timer_create(lv_layout_timer, 150, NULL);
        lv_timer_resume(layout_timer);
    }

    lv_obj_add_event_cb(g_mars_clock_view, gx_home_layout_long_press_handler, LV_EVENT_LONG_PRESSED, NULL);

    lv_wms_init(g_mars_clock_view);
    lv_wms_self_destroy_func_set(g_mars_clock_view, lv_mars_clock_layout_destroy);
    g_cur_clock_view_create_func = lv_mars_clock_layout_create;
    lv_wms_left_create_func_set(g_mars_clock_view, g_cur_list_view_create_func);
    lv_wms_right_create_func_set(g_mars_clock_view, lv_activity_layout_create);
    lv_wms_top_create_func_set(g_mars_clock_view, lv_status_bar_layout_create);
    lv_wms_bottom_create_func_set(g_mars_clock_view, lv_message_layout_create);
    lv_scr_load(g_mars_clock_view);
    lv_timer_resume(layout_timer);

    lvgl_mem_used_dump(__func__, __LINE__);
    return g_mars_clock_view;
}
