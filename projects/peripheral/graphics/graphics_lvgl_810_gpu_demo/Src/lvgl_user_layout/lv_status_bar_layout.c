/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "lv_font.h"
#include "grx_hal.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "lv_user_font.h"
/*
 * SCREEN MACRO DEFINITIONS
 *****************************************************************************************
 */
#define SCR_WEEK_MONDAY                  (1)
#define SCR_WEEK_TUESDAY                 (2)
#define SCR_WEEK_WEDNESDAY               (3)
#define SCR_WEEK_THURSDAY                (4)
#define SCR_WEEK_FRIDAY                  (5)
#define SCR_WEEK_SATURDAY                (6)
#define SCR_WEEK_SUNDAY                  (7)

#define SCR_SECOND_MAX                   (60)
#define SCR_MINUTE_MAX                   (60)
#define SCR_HOUR_MAX                     (24)

#define SCR_WEATHER_CLOUDY               (1)
#define SCR_WEATHER_PARTLY_CLOUDY        (2)
#define SCR_WEATHER_RAINY                (3)
#define SCR_WEATHER_SUNNY                (4)

#define SCR_LOCATION_CHENGDU             (1)
#define SCR_LOCATION_BEIJING             (2)
#define SCR_LOCATION_SHANGHAI            (3)
#define SCR_LOCATION_SHENZHEN            (4)

#define SCR_IMG_INTERVAL                 (40)

#define SCR_BTN_SIZE_X                   (80)
#define SCR_BTN_SIZE_Y                   (80)

/*
 * SCREEN VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_scr_minute = 0;
static uint8_t s_scr_hour = 0;

static lv_style_t s_status_bar_style;
static lv_style_t s_style_btn_grey;
static lv_style_t style_btn_blue;

static lv_obj_t *s_btn_wifi = NULL;
static lv_obj_t *s_btn_bluetooth = NULL;
static lv_obj_t *s_btn_power = NULL;
static lv_obj_t *s_btn_settings = NULL;
static lv_obj_t *s_btn_gps = NULL;
static lv_obj_t *s_btn_call = NULL;
static lv_obj_t *s_btn_home = NULL;

static lv_obj_t *s_time_dot = NULL;
static lv_obj_t *s_time_hour = NULL;
static lv_obj_t *s_time_min = NULL;
static lv_obj_t *s_img_bluetooth = NULL;
static lv_obj_t *s_img_battery = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void status_bar_param_init(void)
{
    s_scr_minute = 7;
    s_scr_hour = 1;
}  

static void status_bar_style_init(void)
{
    lv_style_reset(&s_status_bar_style);
    lv_style_init(&s_status_bar_style);
    lv_style_set_bg_opa(&s_status_bar_style, LV_OPA_COVER);
    lv_style_set_bg_color(&s_status_bar_style, lv_color_black());
    lv_style_set_border_color(&s_status_bar_style, lv_color_black());

    lv_style_reset(&style_btn_blue);
    lv_style_init(&style_btn_blue);
    lv_style_set_bg_color(&style_btn_blue, lv_color_make(0x00, 0x8c, 0xff));

    lv_style_reset(&s_style_btn_grey);
    lv_style_init(&s_style_btn_grey);
    lv_style_set_bg_color(&s_style_btn_grey, lv_color_make(0x44, 0x44, 0x44));
}

static void btn_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* cont = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) 
    {
        lv_color_t color = lv_obj_get_style_bg_color(cont, LV_PART_MAIN);
        lv_color_t color_grey = lv_color_make(0x44, 0x44, 0x44);
        if(color_grey.full == color.full)
        {
            lv_obj_remove_style(cont, &s_style_btn_grey, 0);
            lv_obj_add_style(cont, &style_btn_blue, 0);
        }else{
            lv_obj_remove_style(cont, &style_btn_blue, 0);
            lv_obj_add_style(cont, &s_style_btn_grey, 0);
        }
    }
}

static lv_obj_t* status_bar_btn_create(lv_obj_t *obj, lv_style_t * style)
{
    lv_obj_t* btn = lv_btn_create(obj);
    lv_obj_add_event_cb(btn, btn_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(btn, SCR_BTN_SIZE_X, SCR_BTN_SIZE_Y);
    lv_obj_add_style(btn, style, 0);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0); /*Add a local style too*/
    return btn;
}

static void status_bar_image_create(lv_obj_t *obj, void * src)
{
    lv_obj_t *img = lv_img_create(obj);
    lv_img_set_src(img, src);
    lv_obj_set_style_text_font(img, &lv_font_symbol_40, LV_STATE_DEFAULT);
    lv_obj_center(img);
}

static void status_bar_init(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_status_bar_style, LV_STATE_DEFAULT);
    //time information
    s_time_dot = lv_label_create(obj); // dot
    lv_obj_set_style_text_font(s_time_dot, &lv_font_montserrat_16, LV_STATE_DEFAULT);
    lv_label_set_text_fmt(s_time_dot, ":");
    lv_obj_align(s_time_dot, LV_ALIGN_TOP_MID, 0, 40);
    s_time_hour = lv_label_create(obj); // hour
    lv_obj_set_style_text_font(s_time_hour, &lv_font_montserrat_16, LV_STATE_DEFAULT);
    lv_label_set_text_fmt(s_time_hour, "%02d", s_scr_hour);
    lv_obj_align_to(s_time_hour, s_time_dot, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    s_time_min = lv_label_create(obj); // min
    lv_label_set_text_fmt(s_time_min, "%02d", s_scr_minute);
    lv_obj_set_style_text_font(s_time_min, &lv_font_montserrat_16, LV_STATE_DEFAULT);
    lv_obj_align_to(s_time_min, s_time_dot, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    //statusbar bluetooth link icon
    s_img_bluetooth = lv_img_create(obj);
    lv_img_set_src(s_img_bluetooth, LV_SYMBOL_BELL);
    lv_obj_set_style_text_font(s_img_bluetooth, &lv_font_montserrat_16, LV_STATE_DEFAULT);
    lv_obj_align_to(s_img_bluetooth, s_time_hour, LV_ALIGN_OUT_LEFT_MID, -40, 0);
    //statusbar battery icon
    s_img_battery = lv_img_create(obj);
    lv_img_set_src(s_img_battery, LV_SYMBOL_BATTERY_FULL);
    lv_obj_set_style_text_font(s_img_battery, &lv_font_montserrat_16, LV_STATE_DEFAULT);
    lv_obj_align_to(s_img_battery, s_time_min, LV_ALIGN_OUT_RIGHT_MID, 30, 0);

    //function button
    lv_obj_t* panel = lv_obj_create(obj);
    lv_obj_add_style(panel, &s_status_bar_style, LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(panel, LV_DIR_HOR);
    lv_obj_set_size(panel, 360, 226);
    lv_obj_center(panel);

    s_btn_wifi = status_bar_btn_create(panel, &style_btn_blue);
    lv_obj_align(s_btn_wifi, LV_ALIGN_TOP_LEFT, 10, 0);
    status_bar_image_create(s_btn_wifi, LV_SYMBOL_WIFI); // WIFI

    s_btn_bluetooth = status_bar_btn_create(panel, &s_style_btn_grey);
    lv_obj_align_to(s_btn_bluetooth, s_btn_wifi, LV_ALIGN_OUT_RIGHT_MID, SCR_IMG_INTERVAL, 0);
    status_bar_image_create(s_btn_bluetooth, LV_SYMBOL_BLUETOOTH); // BLUETOOTH

    s_btn_power = status_bar_btn_create(panel, &style_btn_blue);
    lv_obj_align_to(s_btn_power, s_btn_bluetooth, LV_ALIGN_OUT_RIGHT_MID, SCR_IMG_INTERVAL, 0);
    status_bar_image_create(s_btn_power, LV_SYMBOL_POWER); // POWER

    s_btn_settings = status_bar_btn_create(panel, &s_style_btn_grey);
    lv_obj_align_to(s_btn_settings, s_btn_power, LV_ALIGN_OUT_RIGHT_MID, SCR_IMG_INTERVAL, 0);
    status_bar_image_create(s_btn_settings, LV_SYMBOL_SETTINGS); // SETTINGS

    s_btn_gps = status_bar_btn_create(panel, &s_style_btn_grey);
    lv_obj_align_to(s_btn_gps, s_btn_wifi, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);
    status_bar_image_create(s_btn_gps, LV_SYMBOL_GPS); // GPS

    s_btn_call = status_bar_btn_create(panel, &style_btn_blue);
    lv_obj_align_to(s_btn_call, s_btn_gps, LV_ALIGN_OUT_RIGHT_MID, SCR_IMG_INTERVAL, 0);
    status_bar_image_create(s_btn_call, LV_SYMBOL_CALL); // CALL

    s_btn_home = status_bar_btn_create(panel, &s_style_btn_grey);
    lv_obj_align_to(s_btn_home, s_btn_call, LV_ALIGN_OUT_RIGHT_MID, SCR_IMG_INTERVAL, 0);
    status_bar_image_create(s_btn_home, LV_SYMBOL_HOME); // HOME
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t* g_status_bar_view = NULL;
void lv_status_bar_layout_destroy(void)
{
    if(NULL == g_status_bar_view) return;
    lv_wms_deinit(g_status_bar_view);
    lv_obj_del(g_status_bar_view);
    g_status_bar_view = NULL;
}

lv_obj_t* lv_status_bar_layout_create(void)
{
    if(NULL != g_status_bar_view) return g_status_bar_view;
    g_status_bar_view = lv_obj_create(NULL);
    lv_obj_set_scrollbar_mode(g_status_bar_view, LV_SCROLLBAR_MODE_OFF);
    status_bar_param_init();
    status_bar_style_init();
    status_bar_init(g_status_bar_view);

    lv_wms_init(g_status_bar_view);
    lv_wms_self_destroy_func_set(g_status_bar_view, lv_status_bar_layout_destroy);
    lv_wms_bottom_create_func_set(g_status_bar_view, g_cur_clock_view_create_func);

    lvgl_mem_used_dump(__func__, __LINE__);

    lv_scr_load(g_status_bar_view);
    return g_status_bar_view;
}
