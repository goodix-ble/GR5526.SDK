#include <stdio.h>
#include "lv_wms.h"
#include "lv_layout_manager.h"
#include "lv_user_font.h"

#include "layout_scene_manage.h"

#if WMS_VERSION == WMS_VERSION_v2

lv_obj_t *lv_card_status_layout_create(uint32_t win_id);

lv_wms_window_t g_card_status_win = {
    .create_func = lv_card_status_layout_create,
    .window      = NULL,
};

void lv_card_status_layout_destroy(void)
{
    if (!g_card_status_win.window)
    {
        return;
    }

    lv_wms_deinit(g_card_status_win.window);
    lv_obj_del(g_card_status_win.window);
    g_card_status_win.window = NULL;
}

static void lv_event_cb(lv_event_t * e) {
    lv_wms_scene_enter_into(WMS_SCENE_SECOND_ID);
}

lv_obj_t *lv_card_status_layout_create(uint32_t win_id)
{
    if (g_card_status_win.window)
    {
        lv_wms_update_neighbor_setting(g_card_status_win.window, win_id);
        lv_scr_load(g_card_status_win.window);
        return g_card_status_win.window;
    }

    g_card_status_win.window = lv_obj_create(NULL);
    lv_obj_set_size(g_card_status_win.window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_style_bg_color(g_card_status_win.window, lv_palette_main(LV_PALETTE_CYAN), LV_PART_MAIN);
    lv_obj_t *p_widget = lv_label_create(g_card_status_win.window);
    lv_obj_set_style_text_font(p_widget, &lv_font_simsun_28, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_widget, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(p_widget, "STATUS LAYOUT");
    lv_obj_align(p_widget, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(p_widget, lv_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_card_status_win.window, lv_event_cb, LV_EVENT_CLICKED, NULL);

    lv_wms_init(g_card_status_win.window);
    lv_wms_self_destroy_func_set(g_card_status_win.window, lv_card_status_layout_destroy);
    lv_wms_update_neighbor_setting(g_card_status_win.window, win_id);

    lv_scr_load(g_card_status_win.window);
    return g_card_status_win.window;
}

#endif
