#include <stdio.h>
#include "lv_wms.h"
#include "lv_layout_manager.h"
#include "lv_user_font.h"
#include "layout_scene_manage.h"

#include "app_key.h"

#if WMS_VERSION == WMS_VERSION_v2

lv_obj_t * lv_list_layout_create(uint32_t win_id);

lv_wms_window_t g_list_menu_win = {
    .create_func = lv_list_layout_create,
    .window      = NULL,
};


void lv_list_layout_destroy(void)
{
    if (!g_list_menu_win.window)
    {
        return;
    }

    lv_wms_deinit(g_list_menu_win.window);
    lv_obj_del(g_list_menu_win.window);
    g_list_menu_win.window = NULL;
}

//static bool toggle = false;

static void lv_event_cb(lv_event_t * e) {
    lv_wms_scene_enter_into(WMS_SCENE_THIRD_ID);

    //lv_wms_scene_exit_cur_win();

}

static bool key_event_handler(uint32_t key, uint32_t event) {
    bool handled = false;
    printf("LIST UI, Key : %d - %d \r\n", key, event);

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

lv_obj_t * lv_list_layout_create(uint32_t win_id)
{
    if (g_list_menu_win.window)
    {
        lv_wms_update_neighbor_setting(g_list_menu_win.window, win_id);
        lv_scr_load(g_list_menu_win.window);
        return g_list_menu_win.window;
    }

    g_list_menu_win.window = lv_obj_create(NULL);
    lv_obj_set_size(g_list_menu_win.window, DISP_HOR_RES, DISP_VER_RES);

    lv_obj_set_style_bg_color(g_list_menu_win.window, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);

    lv_obj_t *p_widget = lv_label_create(g_list_menu_win.window);
    lv_obj_set_style_text_font(p_widget, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_widget, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(p_widget, "LIST LAYOUT");
    lv_obj_align(p_widget, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(p_widget, lv_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_list_menu_win.window, lv_event_cb, LV_EVENT_CLICKED, NULL);

    lv_wms_init(g_list_menu_win.window);
    lv_wms_self_destroy_func_set(g_list_menu_win.window, lv_list_layout_destroy);
    lv_wms_key_handler_func_set(g_list_menu_win.window, key_event_handler);
    lv_wms_update_neighbor_setting(g_list_menu_win.window, win_id);

    lv_scr_load(g_list_menu_win.window);

    return g_list_menu_win.window;
}

#endif
