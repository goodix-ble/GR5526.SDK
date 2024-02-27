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
 * GLOBAL FUNCTION DECLERATIONS
 *****************************************************************************************
 */
void lv_port_res_mode_set(uint8_t mode);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
LV_IMG_DECLARE(hour);
LV_IMG_DECLARE(minute);
LV_IMG_DECLARE(second);

static lv_obj_t* img_hour = NULL;
static lv_obj_t* img_second = NULL;
static lv_obj_t* img_minute = NULL;
lv_obj_t *img_cube = NULL;
lv_obj_t* g_cube_clock_view = NULL;

#if LV_IMG_PSRAM_CACHE_ENABLE
static lv_img_dsc_t s_wd_img_background;
static void* s_background_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_second;
static void* s_second_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_minute;
static void* s_minute_data = NULL;
static lv_img_dsc_t s_wd_img_live_flower_hour;
static void* s_hour_data = NULL;
#endif

static lv_img_dsc_t  cube_model = {
    .header.always_zero = 0,
    .header.w = 322,
    .header.h = 322,

    .data_size = 322*322*2,

    #if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
    #else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
    #endif
    .data = NULL,
};

//static int g_cube_x = 0;
//static int g_cube_y = 0;
static void* s_cube_data = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static void _draw_cube_side(float *v, int v0, int v1, int v2, int v3, uint32_t col) {
#if 1
    //fill with color
    hal_gfx_fill_quad(v[v0*3], v[v0*3+1],
                   v[v1*3], v[v1*3+1],
                   v[v2*3], v[v2*3+1],
                   v[v3*3], v[v3*3+1], col);
#else
    //blit with box image
    hal_gfx_blit_quad_fit(v[v0*3], v[v0*3+1],
                   v[v1*3], v[v1*3+1],
                   v[v2*3], v[v2*3+1],
                   v[v3*3], v[v3*3+1]);

#endif
}

static void _innercube(int angle_x, int angle_y, int angle_z)
{
    float box_size_2 = 0.1f;
    float FoV = 28.0724869359f;

                   //x     y    z
    float v[]   = {-box_size_2, -box_size_2, box_size_2,   //0  0
                    box_size_2, -box_size_2, box_size_2,   //1  3
                    box_size_2,  box_size_2, box_size_2,   //2  6
                   -box_size_2,  box_size_2, box_size_2,   //3  9
                   -box_size_2, -box_size_2,-box_size_2,   //4  12
                    box_size_2, -box_size_2,-box_size_2,   //5  15
                    box_size_2,  box_size_2,-box_size_2,   //6  18
                   -box_size_2,  box_size_2,-box_size_2};  //7  21

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, (float)322/322, 0.2f, 100.f);

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);
    hal_gfx_mat4x4_rotate_X(proj, angle_x);
    hal_gfx_mat4x4_rotate_Y(proj, angle_y);
    hal_gfx_mat4x4_rotate_Z(proj, angle_z);
    hal_gfx_mat4x4_translate(proj, 0, 0, 2.f-box_size_2);

    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    int i;

    for (i = 0; i < 24; i+=3) {
        float w = 1.f;
        hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, 322, 322,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    //blend color with background
    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE );

    //remove this to draw back sides also
    hal_gfx_tri_cull(HAL_GFX_CULL_NONE);
    _draw_cube_side(v, 0, 1, 2, 3, 0xA08b008b); //front
    _draw_cube_side(v, 4, 0, 3, 7, 0xA0228b22); //left
    _draw_cube_side(v, 1, 5, 6, 2, 0xA0ff4500); //right
    _draw_cube_side(v, 4, 5, 1, 0, 0xA0003366); //top
    _draw_cube_side(v, 3, 2, 6, 7, 0xA066cc99); //bottom
    _draw_cube_side(v, 5, 4, 7, 6, 0x60808080); //back
    hal_gfx_tri_cull(HAL_GFX_CULL_NONE);
}

static void lv_draw_incube_box(void* cube_data)
{
    static int angle_x = 0.f;
    static int angle_y = 0.f;
    static int angle_z = 80.f;

    angle_x = (angle_x+3)%360;
    angle_y = (angle_y+3)%360;
    angle_z = (angle_z+3)%360;

    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cl_bind_circular(&cmd);
    hal_gfx_bind_dst_tex((uint32_t)cube_data, 322, 322, HAL_GFX_RGB565, -1);
    hal_gfx_set_clip(0, 0, 322, 322);
    hal_gfx_clear(0);
    hal_gfx_cl_submit(&cmd);
    hal_gfx_cl_rewind(&cmd);

    _innercube(angle_x, angle_y, angle_z);

    hal_gfx_cl_submit(&cmd);
    hal_gfx_cl_wait(&cmd);

    hal_gfx_cl_le_destroy(&cmd);
}

void lv_layout_draw_period()
{
    if(s_cube_data) lv_draw_incube_box(s_cube_data);
}

static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    if(NULL != s_cube_data){
        app_graphics_mem_free(s_cube_data);
        s_cube_data = NULL;
    }
    if(NULL != s_second_data){
        app_graphics_mem_free(s_second_data);
        s_second_data = NULL;
    }
    if(NULL != s_minute_data){
        app_graphics_mem_free(s_minute_data);
        s_minute_data = NULL;
    }
    if(NULL != s_hour_data){
        app_graphics_mem_free(s_hour_data);
        s_hour_data = NULL;
    }
    lv_wms_go_to_window(lv_clock_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_cube_clock_layout_destroy(void){
    if( NULL == g_cube_clock_view ) return;   
    lv_clk_hand_stop_run();
    lv_wms_deinit(g_cube_clock_view);
    lv_obj_del(g_cube_clock_view);
    g_cube_clock_view = NULL;
}

lv_obj_t* lv_cube_clock_layout_create(void){
    if( NULL != g_cube_clock_view ) return g_cube_clock_view;
    g_cube_clock_view = lv_obj_create(NULL);
    lv_obj_t* obj = g_cube_clock_view;

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

    img_cube = lv_img_create(obj);
    if(NULL == s_cube_data){
        s_cube_data = app_graphics_mem_malloc(cube_model.data_size);
    }
    lv_draw_incube_box(s_cube_data);
    cube_model.data = s_cube_data;
    lv_img_set_src(img_cube, &cube_model);
    lv_obj_set_pos(img_cube, 61, 61);

    img_second = lv_img_create(obj);
    /***save the second in PSRAM for faster speed***************************************************/
    #if LV_IMG_PSRAM_CACHE_ENABLE
    memcpy(&s_wd_img_live_flower_second, &wd_img_live_flower_second, sizeof(lv_img_dsc_t));
    if( NULL == s_second_data)
    {
        s_second_data = app_graphics_mem_malloc(wd_img_live_flower_second.data_size);
    }
    lv_port_res_mode_set(2);
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
    #endif
    /**********************************************************************************************/

    lv_obj_set_pos(img_hour, 96 + 130, 96 + 130 - 10);
    lv_img_set_pivot(img_hour, 0, 10);

    lv_obj_t *img_center = lv_img_create(obj);
    lv_img_set_src(img_center, &wd_img_live_flower_center);
    lv_obj_set_pos(img_center, 96 + 130 - 10, 96 + 130 - 10);
    lv_obj_update_layout(obj); // key for the layout show correctly

    lv_clk_set_hour_hand(img_hour);
    lv_clk_set_min_hand(img_minute);
    lv_clk_set_sec_hand(img_second);
    lv_clk_set_bg_cb(NULL, lv_layout_draw_period);
    lv_obj_add_event_cb(g_cube_clock_view, gx_home_layout_long_press_handler, LV_EVENT_LONG_PRESSED, NULL);
    lv_clk_hand_start_run();

    lv_wms_init(g_cube_clock_view);
    lv_wms_self_destroy_func_set(g_cube_clock_view, lv_cube_clock_layout_destroy);
    g_cur_clock_view_create_func = lv_cube_clock_layout_create;
    lv_scr_load(g_cube_clock_view);
    lv_wms_left_create_func_set(g_cube_clock_view, g_cur_list_view_create_func);
    lv_wms_right_create_func_set(g_cube_clock_view, lv_activity_layout_create);
    lv_wms_top_create_func_set(g_cube_clock_view, lv_status_bar_layout_create);
    lv_wms_bottom_create_func_set(g_cube_clock_view, lv_message_layout_create);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_cube_clock_view;
}
