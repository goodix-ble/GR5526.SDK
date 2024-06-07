#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_layout_app_menu.h"

#define LIST_MENU_TITLE_HEIGHT      70
#define ISTR_MARK(TXT)              TXT
#define ISTR(TXT)                   TXT

static const char* APPLIST_STYLE_OPTIONS = ISTR_MARK("Vertical\nHorizontal");

static lv_style_t s_roller_style;
static lv_style_t s_roller_selected_style;

extern void lv_layout_set_cardflow_style(bool is_hori);
extern bool lv_layout_get_cardflow_style(void);

static void roller_event_handler(lv_event_t *evt)
{
    uint16_t selected = lv_roller_get_selected(evt->current_target);

    switch(selected) {
        case 0:
        default:
        {
            lv_layout_set_cardflow_style(false);
        }
        break;

        case 1:
        {
            lv_layout_set_cardflow_style(true);
        }
        break;
    }
}

extern lv_obj_t * lv_layout_setting_create_common_title(lv_obj_t * parent, lv_font_t * font, char * text);

lv_obj_t * lv_layout_setting_cardflow_style_create(lv_obj_t * parent)
{
    lv_obj_t *p_window = lv_layout_setting_create_common_title(parent, &lv_font_montserrat_30, "Cardflow Style");

    // Roller container
    lv_obj_t *p_container = lv_obj_create(p_window);
    lv_obj_set_size(p_container, DISP_HOR_RES, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT - 28);
    lv_obj_set_pos(p_container, 0, LIST_MENU_TITLE_HEIGHT + 28);

    // Roller
    lv_obj_t *p_roller = lv_roller_create(p_container);
    lv_obj_set_size(p_roller, DISP_HOR_RES, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT - 28);
    lv_obj_set_pos(p_roller, 0, -20);
    lv_obj_set_style_bg_color(p_roller, lv_color_black(), 0);
    lv_roller_set_options(p_roller, ISTR(APPLIST_STYLE_OPTIONS), LV_ROLLER_MODE_NORMAL);
    lv_obj_add_event_cb(p_roller, roller_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_roller_set_selected(p_roller, lv_layout_get_cardflow_style(), LV_ANIM_OFF);

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

    return p_window;
}

