/**
 *****************************************************************************************
 *
 * @file lv_layout_clock.c
 *
 * @brief lvgl clock Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2021 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

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
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t* img_hour;
static lv_obj_t* img_second;
static lv_obj_t* img_minute;
lv_obj_t* g_black_clock_view = NULL;

/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_simsun_28;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gx_home_layout_long_press_handler(lv_event_t * e)
{
    lv_wms_go_to_window(lv_clock_view_select_create);
    lv_indev_wait_release(lv_indev_get_act());
}


void lv_black_clock_layout_destroy()
{
    if(NULL == g_black_clock_view) return;
    lv_clk_hand_stop_run();
    lv_wms_deinit(g_black_clock_view);
    lv_obj_del(g_black_clock_view);
    g_black_clock_view = NULL;
}

lv_obj_t* lv_black_clock_layout_create()
{
    if(NULL != g_black_clock_view) return g_black_clock_view;
    g_black_clock_view = lv_obj_create(NULL);
    lv_obj_t *img_background = lv_img_create(g_black_clock_view);
    lv_img_set_src(img_background, &wd_img_watch_face_2);
    // Notice the background image is RGB565 format
    lv_obj_set_pos(img_background, 0, 0);

    lv_obj_t* activity_obj = lv_img_create(g_black_clock_view);
    lv_img_set_src(activity_obj, &wd_img_activity);
    lv_obj_set_pos(activity_obj, 90, 182);
    
    lv_obj_t* sun_obj = lv_img_create(g_black_clock_view);
    lv_img_set_src(sun_obj, &wd_img_sun2);
    lv_obj_set_pos(sun_obj, 210, 305);
    
    lv_obj_t* step_obj = lv_img_create(g_black_clock_view);
    lv_img_set_src(step_obj, &wd_img_step2);
    lv_obj_set_pos(step_obj, 305, 182);
    
    lv_obj_t* label = lv_label_create(g_black_clock_view);
    lv_obj_set_style_text_font(label, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label, "30°");
    lv_obj_set_pos(label, 212, 337);

    lv_obj_t* label4 = lv_label_create(g_black_clock_view);
    lv_obj_set_style_text_font(label4, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label4, "25600");
    lv_obj_set_pos(label4, 287, 228);
    
    lv_obj_t* label2 = lv_label_create(g_black_clock_view);
    lv_obj_set_style_text_font(label2, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label2, "5月18日");
    lv_obj_set_pos(label2, 145, 115);

    lv_obj_t* label3 = lv_label_create(g_black_clock_view);
    lv_obj_set_style_text_font(label3, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_label_set_text(label3, "星期三");
    lv_obj_set_pos(label3, 240, 115);

    img_second = lv_img_create(g_black_clock_view);
    lv_img_set_src(img_second, &wd_img_watch2_second1);    
    lv_obj_set_pos(img_second, 202, 224);

    img_minute = lv_img_create(g_black_clock_view);
    lv_img_set_src(img_minute, &wd_img_watch2_minite1);
    lv_obj_set_pos(img_minute, 227, 220); 

    img_hour = lv_img_create(g_black_clock_view);
    lv_img_set_src(img_hour, &wd_img_watch2_hour1);
    lv_obj_set_pos(img_hour, 227, 220);

    lv_obj_t *img_center = lv_img_create(g_black_clock_view);
    lv_img_set_src(img_center, &wd_img_watch2_center_point);
    lv_obj_set_pos(img_center, 216, 216);

    lv_img_set_pivot(img_second, 25, 3);
    lv_img_set_pivot(img_minute, 0, 7);
    lv_img_set_pivot(img_hour, 0, 7);

    lv_obj_update_layout(g_black_clock_view);
    lv_img_set_angle(img_hour, 400);
    lv_img_set_angle(img_minute, 600);

    lv_clk_set_hour_hand(img_hour);
    lv_clk_set_min_hand(img_minute);
    lv_clk_set_sec_hand(img_second);
    lv_clk_set_bg_cb(NULL, NULL);
    lv_clk_hand_start_run();
    lv_obj_add_event_cb(g_black_clock_view, gx_home_layout_long_press_handler, LV_EVENT_LONG_PRESSED, NULL);

    lv_wms_init(g_black_clock_view);
    lv_wms_self_destroy_func_set(g_black_clock_view, lv_black_clock_layout_destroy);
    g_cur_clock_view_create_func = lv_black_clock_layout_create;
    lv_scr_load(g_black_clock_view);
    lv_wms_left_create_func_set(g_black_clock_view, g_cur_list_view_create_func);
    lv_wms_right_create_func_set(g_black_clock_view, lv_activity_layout_create);
    lv_wms_top_create_func_set(g_black_clock_view, lv_status_bar_layout_create);
    lv_wms_bottom_create_func_set(g_black_clock_view, lv_message_layout_create);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_black_clock_view;
}
