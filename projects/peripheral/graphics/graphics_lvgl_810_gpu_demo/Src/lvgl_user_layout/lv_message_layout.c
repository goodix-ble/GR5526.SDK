/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "hal_gfx_cmdlist.h"
/*
 * DEFINE
 *****************************************************************************************
 */
#define CANVAS_LAYOUT        1

/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */
extern lv_font_t lv_font_simsun_22;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
 LV_IMG_DECLARE(wd_img_alipay);
LV_IMG_DECLARE(wd_img_netesae);
LV_IMG_DECLARE(wd_img_workhours);
LV_IMG_DECLARE(wd_img_phone);
LV_IMG_DECLARE(wd_img_calander); 
LV_IMG_DECLARE(wd_img_heartrate);
LV_IMG_DECLARE(wd_img_events);
const lv_img_dsc_t* message_icon_addrs[] = 
{
    &wd_img_wechat,
    &wd_img_alipay,
    &wd_img_netesae,
    &wd_img_workhours,
    &wd_img_phone,
    &wd_img_wechat,
    &wd_img_calander,
    &wd_img_heartrate,
    &wd_img_events,
    &wd_img_wechat,
    &wd_img_alipay,
    &wd_img_netesae,
    &wd_img_workhours,
    &wd_img_phone,
    &wd_img_wechat,
    &wd_img_calander,
    &wd_img_heartrate,
    &wd_img_events,
    &wd_img_phone,
    &wd_img_wechat,
};

static const char message_head_msg[][20]= 
{
    "微信",
    "支付宝",
    "网易云音乐",
    "运动",
    "微信",
    "电话",
    "日历",
    "心率",
    "日程安排",
    "微信",
    "支付宝",
    "网易云音乐",
    "运动",
    "微信",
    "支付宝",
    "微信",
    "支付宝",
    "运动",
    "微信",
    "电话",
};

static const char message_time_msg[][10]= 
{
    "10:31",
    "10:32",
    "10:33",
    "10:34",
    "10:35",
    "10:36",
    "10:37",
    "10:38",
    "10:39",
    "10:40",
    "10:41",
    "10:42",
    "10:43",
    "10:44",
    "10:45",
    "10:46",
    "10:47",
    "10:48",
    "10:49",
    "10:50",
};

static const char message_msg[][50]= 
{
    "[4条]妈妈:你这周末回家吗？",
    "你3月份的花呗消费1234元",
    "123456789101112...",
    "恭喜你今天的运动已达标!",
    "晓米:今天天气不错，出去晒太...",
    "未接来电18345678901",
    "今天是爸爸的生日，记得打电话...",
    "你今天的心率有点偏高，请注意...",
    "你今天下午2:00有会议",
    "[3条]看理想：愿四方安宁，一心...",
    "您当前信用优良，芝麻分数为...",
    "4月黑胶等级权益-云贝可以领...",
    "你今天偷懒了哦，赶紧动起来...",
    "Goodix：今天食堂有上新菜...",
    "你2月份的花呗还未还清，请尽...",
    "[2条]妈妈:我给你寄了吃的，记...",
    "你3月份支出共3245元",
    "今天的运行目标还有20%，加油...",
    "小明：今天有空吗？",
    "未接来电18254678901",
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t* g_message_layout_view = NULL;
#if CANVAS_LAYOUT
uint8_t* _p_buffer[9] = {0};
#endif

void lv_message_layout_destroy(void)
{
    if(NULL == g_message_layout_view) return;
    lv_wms_deinit(g_message_layout_view);
    lv_obj_del(g_message_layout_view);
    g_message_layout_view = NULL;
#if CANVAS_LAYOUT
    for (uint32_t i = 0; i < 9; i++){
        if(NULL != _p_buffer[i]){
            app_graphics_mem_free(_p_buffer[i]);
            _p_buffer[i] = NULL;
        }
    }
#endif
}

lv_obj_t* lv_message_layout_create(void)
{
    if(NULL != g_message_layout_view) return g_message_layout_view;
    g_message_layout_view = lv_obj_create(NULL);
    lv_obj_set_scrollbar_mode(g_message_layout_view, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(g_message_layout_view, 454, 454);
    lv_obj_set_flex_flow(g_message_layout_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_message_layout_view, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    static lv_style_t message_style = {0};
    lv_style_reset(&message_style);
    lv_style_init(&message_style);
    lv_style_set_text_font(&message_style, &lv_font_simsun_22);
    lv_style_set_bg_opa(&message_style, LV_OPA_COVER);
    lv_style_set_bg_color(&message_style, lv_palette_darken(LV_PALETTE_GREY, 4));
#if !CANVAS_LAYOUT
    for (uint32_t i = 0; i < 20; i++){
        lv_obj_t* obj = lv_obj_create(g_message_layout_view);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(obj, 350, 100);
        lv_obj_add_style(obj, &message_style, 0);
        lv_obj_t* img = lv_img_create(obj);
        lv_img_set_src(img, message_icon_addrs[i]);
        lv_obj_t* lab = lv_label_create(obj);
        lv_obj_align_to(lab, img, LV_ALIGN_LEFT_MID, 34, 0);
        lv_label_set_text(lab, message_head_msg[i]);

        lv_obj_t* lab1 = lv_label_create(obj);
        lv_obj_align_to(lab1, lab, LV_ALIGN_LEFT_MID, 240, 0);
        lv_label_set_text(lab1, message_time_msg[i]);
 
        lv_obj_t* lab2 = lv_label_create(obj);
        lv_obj_align_to(lab2, img, LV_ALIGN_LEFT_MID, 0, 34);
        lv_label_set_text(lab2, message_msg[i]);
    }
#else
    lv_draw_img_dsc_t img_dsc;
    lv_draw_img_dsc_init(&img_dsc);
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.font = &lv_font_simsun_22;

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 15;
    rect_dsc.blend_mode = LV_BLEND_MODE_NORMAL;
    rect_dsc.bg_color = lv_palette_darken(LV_PALETTE_GREY, 4);
    rect_dsc.bg_opa = LV_OPA_COVER;
 
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t* cl = &cmd;
    hal_gfx_cl_bind_circular(cl);
    for (uint32_t i = 0; i < 9; i++){

        lv_obj_t * canvas = lv_canvas_create(g_message_layout_view);
        if(NULL == _p_buffer[i]){
            _p_buffer[i] = app_graphics_mem_malloc(350 * 100 * 2);
        }
        memset(_p_buffer[i], 0x00, 350 * 100 * 2);
        lv_canvas_set_buffer(canvas, _p_buffer[i], 350, 100, LV_IMG_CF_GDX_RGB565);

        lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(canvas, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_set_size(canvas, 350, 100);
        //lv_obj_set_pos(canvas, 200, 0);
        lv_canvas_draw_rect(canvas, 0, 0, 350, 100, &rect_dsc);
        lv_canvas_draw_img(canvas, 10, 10, message_icon_addrs[i], &img_dsc);
        lv_canvas_draw_text(canvas, 40, 10, 200, &label_dsc, message_head_msg[i]);
        lv_canvas_draw_text(canvas, 280, 10, 60, &label_dsc, message_time_msg[i]);
        lv_canvas_draw_text(canvas, 10, 40, 350, &label_dsc, message_msg[i]);
    }
    hal_gfx_cl_destroy(cl);
#endif

    lv_wms_init(g_message_layout_view);
    lv_wms_self_destroy_func_set(g_message_layout_view, lv_message_layout_destroy);
    lv_wms_top_create_func_set(g_message_layout_view, g_cur_clock_view_create_func);

    lvgl_mem_used_dump(__func__, __LINE__);

    lv_scr_load(g_message_layout_view);
    return g_message_layout_view;
}
