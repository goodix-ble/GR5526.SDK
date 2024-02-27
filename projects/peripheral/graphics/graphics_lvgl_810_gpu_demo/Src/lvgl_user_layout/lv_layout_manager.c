/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lv_port_disp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app_qspi.h"
#include "app_rtos_cfg.h"
#include "graphics_defs.h"
#include "graphics_dc_lcd_drv.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_font.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"
#include "lv_port_indev.h"
#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "FreeRtos.h"
#include "task.h"
#include "lv_layout_manager.h"
#include "disp_driver.h"
#include "app_graphics_mem.h"
#include "lv_conf.h"
#include "lv_wms.h"

#define MEM_USED_SIZE  (lv_mem_used_size_get())
#define TICK_USED_CNT  (xTaskGetTickCount() - start_tick_cnt)
#define MONKEY_ENABLE  (0)

uint32_t lv_mem_used_size_get(void){
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    uint32_t used_size = mon.total_size - mon.free_size;
    return used_size;
}

void lv_home_layout_key_event(void){
    static bool main_switch = true;
    if (main_switch){
        lv_wms_go_to_window(g_cur_list_view_create_func);
    }else{
        lv_wms_go_to_window(g_cur_clock_view_create_func);
    }
    main_switch = !main_switch;
}

#if (1 == MONKEY_ENABLE)
uint32_t g_win_id = 0;
const lv_create_func_t _monkey_wins[] = {
    lv_flower_clock_layout_create,
    lv_black_clock_layout_create,
    lv_mars_clock_layout_create,
    lv_activity_layout_create,
    lv_heartrate_layout_create,
    lv_chart_layout_create,
    lv_status_bar_layout_create,
    lv_message_layout_create,
    lv_nebula_layout_create,
    lv_cube_clock_layout_create,
    lv_scale_list_layout_create,
    lv_setting_view_create,
    lv_list_view_select_create,
    lv_clock_view_select_create,
    lv_swtich_list_setting_view_create,
    lv_volume_adjust_layout_create,
    lv_graphics_circle_layout_create,
    lv_graphics_cube_layout_create,
    lv_graphics_arc_layout_create,
    lv_graphics_jump_layout_create,
    lv_graphics_stencil_layout_create,
    lv_graphics_tsc4_layout_create,
    lv_widgets_test_view_create,
};

void lv_monkey_timer(lv_timer_t * tmr){
    uint32_t win_num = sizeof(_monkey_wins)/sizeof(lv_create_func_t);
    g_win_id = lv_rand(0, (win_num-1));
    lv_wms_go_to_window(_monkey_wins[g_win_id]);
}
#endif

void lv_home_layout_init(void)
{
#if (0 == MONKEY_ENABLE)
    // record the start tick count when home init
    uint32_t start_tick_cnt = xTaskGetTickCount();
    printf("LVGL Init -> (Size: %d, Tick: %d)\n\n", MEM_USED_SIZE, TICK_USED_CNT);

    // destroy the default screen to save memory because we never used it
    if(lv_scr_act()) lv_obj_del(lv_scr_act());

    // show the tick count and mem used size when home init
    printf("Home Init -> (Size: %d, Tick: %d)\n\n", MEM_USED_SIZE, TICK_USED_CNT);

#if WMS_VERSION == WMS_VERSION_v2
    extern void lv_wms_scene_init(void) ;
    lv_wms_scene_init();
#else
    // show the flower clock view
    g_cur_list_view_create_func = lv_scale_list_layout_create;
    lv_flower_clock_layout_create();
#endif

    printf("Flower Clock Init -> (Size: %d, Tick: %d)\n\n", MEM_USED_SIZE, TICK_USED_CNT);
#else
    g_cur_list_view_create_func = lv_scale_list_layout_create;
    lv_timer_create(lv_monkey_timer, 1000, NULL);
    lv_port_debug_info_enable(true);
#endif
}
