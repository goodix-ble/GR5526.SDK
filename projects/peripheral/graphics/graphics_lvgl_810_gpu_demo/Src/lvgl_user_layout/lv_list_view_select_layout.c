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
#include "lv_scale_list.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
lv_create_func_t g_cur_list_view_create_func = NULL;

/**********************
 *      DECLARATIONS
 **********************/

lv_obj_t* lv_scale_list_layout_get(void);

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* func_select_obj = NULL;

void lv_list_view_select_destroy(void)
{
    if (func_select_obj == NULL) return;
    lv_wms_deinit(func_select_obj);
    lv_obj_del(func_select_obj);
    func_select_obj = NULL;
}

static void gx_simple_list_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_scale_list_layout_create);
    lv_scale_list_set_mode(lv_scale_list_layout_get(), SCALE_NORMAL_MODE);
}

static void gx_circle_list_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_scale_list_layout_create);
    lv_scale_list_set_mode(lv_scale_list_layout_get(), SCALE_CIRCLE_MODE);
}

static void gx_star_list_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_nebula_layout_create);
}

static void gx_nine_grid_click_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_scale_list_layout_create);
    lv_scale_list_set_mode(lv_scale_list_layout_get(), SCALE_GRID_MODE);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static const lv_img_dsc_t* img_array[] =
{
    &wd_img_list3_thumbnail,
    &wd_img_list1_thumbnail,
    &wd_img_star_thumbnail,
    &wd_img_nine_grid_thumbnail
};

typedef void (*gx_click_handler_t)(lv_event_t * e);
static gx_click_handler_t gx_click_handler[] = 
{
    gx_simple_list_click_handler,
    gx_circle_list_click_handler,
    gx_star_list_click_handler,
    gx_nine_grid_click_handler,
};

#define DIAL_SELECT_WIDTH  330
#define DIAL_SELECT_HEIGHT 454
#define DIAL_ARC_SIZE      300
lv_obj_t* lv_list_view_select_create(void)
{
    if (func_select_obj == NULL){
        func_select_obj = lv_obj_create(NULL);

        lv_obj_set_style_bg_color(func_select_obj, lv_color_black(), 0);
        lv_obj_set_scroll_snap_x(func_select_obj, LV_SCROLL_SNAP_CENTER);
        lv_obj_add_flag(func_select_obj, LV_OBJ_FLAG_SCROLL_ONE);

        for (uint32_t i = 0; i < 4; i++)
        {
            lv_obj_t* obj = lv_obj_create(func_select_obj);
            lv_obj_set_size(obj, DIAL_SELECT_WIDTH, DIAL_SELECT_HEIGHT);
            lv_obj_set_pos(obj, DIAL_SELECT_WIDTH * i, 0);
            lv_obj_t *img = lv_img_create(obj);
            lv_img_set_src(img, img_array[i]);
            lv_obj_center(img);
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
            lv_obj_center(arc);
        }
    }
    lv_scr_load(func_select_obj);
    lv_wms_init(func_select_obj);
    lv_wms_self_destroy_func_set(func_select_obj, lv_list_view_select_destroy);
    lvgl_mem_used_dump(__func__, __LINE__);
    return func_select_obj;
}
