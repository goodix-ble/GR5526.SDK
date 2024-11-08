#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_layout_app_menu.h"

#define LIST_MENU_TITLE_HEIGHT      70
#define ISTR_MARK(TXT)              TXT
#define ISTR(TXT)                   TXT

static const char* APPLIST_STYLE_OPTIONS = ISTR_MARK("Linear List\nCircular List\nSphere Grid\nTriform Grid\nDialer");

static lv_style_t s_roller_style;
static lv_style_t s_roller_selected_style;

static void roller_event_handler(lv_event_t *evt)
{
    app_menu_set_style((app_menu_style_t)lv_roller_get_selected(evt->current_target));
}

extern lv_obj_t * lv_layout_setting_create_common_title(lv_obj_t * parent, lv_font_t * font, char * text);

static uint16_t _setting_list_style_get(void) {
    return (uint16_t)app_menu_get_style();
}


lv_obj_t * lv_layout_setting_list_style_create(lv_obj_t * parent)
{
    lv_obj_t *p_window = lv_layout_setting_create_common_title(parent, &lv_font_montserrat_30, "List Style");
    lv_obj_set_style_bg_color(p_window, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(p_window, 255, 0);

    // Roller container
    lv_obj_t *p_container = lv_obj_create(p_window);
    lv_obj_set_size(p_container, DISP_HOR_RES, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT - 28);
    lv_obj_set_pos(p_container, 0, LIST_MENU_TITLE_HEIGHT + 28);

    // Roller
    lv_obj_t *p_roller = lv_roller_create(p_container);
    lv_obj_set_size(p_roller, DISP_HOR_RES, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT - 28);
    lv_obj_set_pos(p_roller, 0, -120);
    lv_obj_set_style_bg_color(p_roller, lv_color_black(), 0);
    lv_roller_set_options(p_roller, ISTR(APPLIST_STYLE_OPTIONS), LV_ROLLER_MODE_NORMAL);
    lv_obj_add_event_cb(p_roller, roller_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_roller_set_selected(p_roller, _setting_list_style_get(), LV_ANIM_OFF);

    // Roller Style
    lv_style_init(&s_roller_style);
    lv_style_set_anim_time(&s_roller_style, 200);
    lv_style_set_text_color(&s_roller_style, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_set_text_font(&s_roller_style, &lv_font_montserrat_48_gdx);
    lv_style_set_text_line_space(&s_roller_style, 30);
    lv_style_set_border_width(&s_roller_style, 0);
    lv_style_set_text_align(&s_roller_style, LV_TEXT_ALIGN_CENTER);

    // Roller Selected Style
    lv_style_init(&s_roller_selected_style);
    lv_style_set_bg_color(&s_roller_selected_style, lv_palette_darken(LV_PALETTE_GREY, 4));
    lv_style_set_text_color(&s_roller_selected_style, lv_palette_lighten(LV_PALETTE_CYAN, 1));
    lv_style_set_text_font(&s_roller_selected_style, &lv_font_montserrat_48_gdx);
    lv_style_set_text_align(&s_roller_selected_style, LV_TEXT_ALIGN_CENTER);

    lv_obj_add_style(p_roller, &s_roller_style, LV_PART_MAIN);
    lv_obj_add_style(p_roller, &s_roller_selected_style, LV_PART_SELECTED);

    lv_roller_set_visible_row_count(p_roller, 5);

    return p_window;
}
