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

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
lv_create_func_t g_cur_clock_view_create_func = NULL;

extern lv_font_t lv_font_msyhbd_40;
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* home_select_obj = NULL;
void lv_clock_view_select_destroy(void)
{
    if(NULL == home_select_obj) return;
    lv_wms_deinit(home_select_obj);
    lv_obj_del(home_select_obj);
    home_select_obj = NULL;
}

static void gx_flower_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_flower_clock_layout_create);
}

static void gx_mars_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_mars_clock_layout_create);
}

static void gx_black_clock_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_black_clock_layout_create);
}

static void gx_cube_clock_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_cube_clock_layout_create);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static const char str_infos[][15]= 
{
    "花开荼靡", "火星探索","黑色静谧", "运动时尚",
};

static const lv_img_dsc_t* img_array[] =
{
    &wd_img_flower_thumbnail,
    &wd_img_mars_thumbnail,
    &wd_img_dark_thumbnail,
    &wd_img_activity1_thumbnail,
};

typedef void (*gx_click_handler_t)(lv_event_t * e);
static gx_click_handler_t gx_click_handler[] = 
{
    gx_flower_click_handler,
    gx_mars_click_handler,
    gx_black_clock_handler,
    gx_cube_clock_handler,
};

#define DIAL_SELECT_WIDTH  330
#define DIAL_SELECT_HEIGHT 454
#define DIAL_ARC_SIZE      300
lv_obj_t* lv_clock_view_select_create(void)
{
    if(NULL == home_select_obj){
        home_select_obj = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(home_select_obj, lv_color_black(), 0);
        lv_obj_set_scroll_snap_x(home_select_obj, LV_SCROLL_SNAP_CENTER);
        lv_obj_add_flag(home_select_obj, LV_OBJ_FLAG_SCROLL_ONE);
        for (uint32_t i = 0; i < 4; i++)
        {
            lv_obj_t* obj = lv_obj_create(home_select_obj);
            lv_obj_set_size(obj, DIAL_SELECT_WIDTH, DIAL_SELECT_HEIGHT);
            lv_obj_set_pos(obj, DIAL_SELECT_WIDTH * i + 62, 0);
            lv_obj_t* label = lv_label_create(obj);
            lv_obj_set_style_text_font(label, &lv_font_msyhbd_40, 0);
            lv_label_set_text(label, str_infos[i]);
            lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 15);
            
            lv_obj_t *img = lv_img_create(obj);
            lv_img_set_src(img, img_array[i]);
            lv_obj_align(img, LV_ALIGN_CENTER, 0, 15);
            
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);  /*To not allow adjusting by click*/
            lv_obj_add_event_cb(obj, gx_click_handler[i], LV_EVENT_CLICKED, NULL);

            lv_obj_t* arc = lv_arc_create(obj);
            lv_obj_set_size(arc, DIAL_ARC_SIZE, DIAL_ARC_SIZE);
            lv_arc_set_bg_angles(arc, 0, 360);
            lv_obj_set_style_arc_width(arc, 5, LV_PART_MAIN);
            lv_obj_set_style_arc_color(arc, lv_color_white(), 0);
            lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
            lv_obj_remove_style(arc, NULL, LV_PART_INDICATOR);   /*Be sure the knob is not displayed*/
            lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
            lv_obj_align(arc, LV_ALIGN_CENTER, 0, 15);
        }
    }
    lv_scr_load(home_select_obj);
    lv_wms_init(home_select_obj);
    lv_wms_self_destroy_func_set(home_select_obj, lv_clock_view_select_destroy);
    lvgl_mem_used_dump(__func__, __LINE__);
    return home_select_obj;
}
