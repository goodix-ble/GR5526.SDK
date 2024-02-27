#include <stdio.h>
#include "lv_wms.h"
#include "lv_layout_manager.h"
#include "lv_user_font.h"
#include "layout_scene_manage.h"
#include "app_key.h"

#if WMS_VERSION == WMS_VERSION_v2

lv_obj_t * lv_sports_layout_create(uint32_t win_id);

lv_wms_window_t g_sports_win = {
    .create_func = lv_sports_layout_create,
    .window      = NULL,
};


void lv_sports_layout_destroy(void)
{
    if (!g_sports_win.window)
    {
        return;
    }

    lv_wms_deinit(g_sports_win.window);
    lv_obj_del(g_sports_win.window);
    g_sports_win.window = NULL;
}

static bool key_event_handler(uint32_t key, uint32_t event) {
    bool handled = false;
    printf("SPORTS UI, Key : %d - %d \r\n", key, event);

    switch(event) {
        case APP_KEY_SINGLE_CLICK:
        {
            handled = true;
            lv_wms_scene_exit_cur_win();
        }
        break;
    }

    return handled;
}

lv_obj_t * lv_sports_layout_create(uint32_t win_id)
{
    if (g_sports_win.window)
    {
        lv_wms_update_neighbor_setting(g_sports_win.window, win_id);
        lv_scr_load(g_sports_win.window);
        return g_sports_win.window;
    }

    g_sports_win.window = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_sports_win.window, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_sports_win.window, lv_palette_main(LV_PALETTE_DEEP_ORANGE), LV_PART_MAIN);
    lv_obj_set_size(g_sports_win.window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_t *p_widget = lv_label_create(g_sports_win.window);
    lv_obj_set_style_text_font(p_widget, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_widget, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(p_widget, "SPORTS LAYOUT");
    lv_obj_align(p_widget, LV_ALIGN_CENTER, 0, 0);

    lv_wms_init(g_sports_win.window);
    lv_wms_self_destroy_func_set(g_sports_win.window, lv_sports_layout_destroy);
    lv_wms_key_handler_func_set(g_sports_win.window, key_event_handler);
    lv_wms_update_neighbor_setting(g_sports_win.window, win_id);

    lv_scr_load(g_sports_win.window);

    return g_sports_win.window;
}

#endif
