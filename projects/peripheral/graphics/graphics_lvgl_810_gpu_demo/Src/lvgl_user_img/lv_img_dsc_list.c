#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
const lv_img_dsc_t  wd_img_10_upgrade= {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_10_UPGRADE),
    };
const lv_img_dsc_t  wd_img_11_About = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_11_ABOUT),
    };
const lv_img_dsc_t  wd_img_12_taobao = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_15_ALIPAY),
    };
const lv_img_dsc_t  wd_img_13_entertainment = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_14_phonebook = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_18_WECHAT),
    };
const lv_img_dsc_t  wd_img_15_Googlemap = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_19_BREATHING),
    };
const lv_img_dsc_t  wd_img_16_baidu = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_20_WEATHER),
    };
const lv_img_dsc_t  wd_img_17_iqiyi = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_21_FLASHLIGHT),
    };
const lv_img_dsc_t  wd_img_18_netease = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_22_EVENTS),
    };
const lv_img_dsc_t  wd_img_19_wallet = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_28_WALLET),
    };
const lv_img_dsc_t  wd_img_1_App_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_APP_VIEW),
    };
const lv_img_dsc_t  wd_img_1_setting = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_1_SETTINGS),
    };
const lv_img_dsc_t  wd_img_20_music = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_21_calendar = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_23_CALENDAR),
    };
const lv_img_dsc_t  wd_img_22_Chrome = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_34_RECORDER),
    };
const lv_img_dsc_t  wd_img_23_twitter = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_25_HIMALAYA),
    };
const lv_img_dsc_t  wd_img_24_WhatsApp = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_26_FIND_PHONE),
    };
const lv_img_dsc_t  wd_img_25_cmbc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_27_WORLD_CLOCK),
    };
const lv_img_dsc_t  wd_img_26_online_class = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_2_FACE_GALLERY),
    };
const lv_img_dsc_t  wd_img_27_message = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_36_TIMER),
    };
const lv_img_dsc_t  wd_img_28_Google_meetting = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_3_HEART_RATE),
    };
const lv_img_dsc_t  wd_img_29_facebook = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_4_DAILY_ACTIVITY),
    };
const lv_img_dsc_t  wd_img_draw_cube = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_5_DRAW_CUBE),
    };
const lv_img_dsc_t  wd_img_draw_arcs = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_6_DRAW_ARCS),
    };
const lv_img_dsc_t  wd_img_jumping = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_7_JUMPING),
    };
const lv_img_dsc_t  wd_img_draw_circle = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_8_DRAW_CIRCLE),
    };

const lv_img_dsc_t  wd_img_mask_stencil = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_9_MASK_STENCIL),
    };
const lv_img_dsc_t  wd_img_compress = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_37_COMPRESS),
    };
const lv_img_dsc_t  wd_img_messages = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_11_MESSAGES),
    };
const lv_img_dsc_t  wd_img_stress = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_12_STRESS),
    };

const lv_img_dsc_t  wd_img_2_alipay = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_2_function_key = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_FUNCTION_KEY),
    };
const lv_img_dsc_t  wd_img_30_douban = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_31_QQ_space = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_32_google_plus = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_33_weather = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_34_google_play = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_35_application_store = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_36_icbc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_37_QQ_music = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_38_file_manage = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_39_QQ = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_3_clock_setting = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_3_CLOCK_SETTING),
    };
const lv_img_dsc_t  wd_img_3_wechat = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_40_yahoo = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_4_favorite = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_4_FAVORITE),
    };
const lv_img_dsc_t  wd_img_4_heart = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_5_dianping = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_5_notice_manage = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_5_NOTICE_MANAGE),
    };
const lv_img_dsc_t  wd_img_6_camera = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_6_system_update = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_6_SYSTEM_UPDATE),
    };
const lv_img_dsc_t  wd_img_7_general_setting = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_7_GENERAL_SETTING),
    };
const lv_img_dsc_t  wd_img_7_tiktok = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_8_activity = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_8_auxiliary_means = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_8_AUXILIARY_MEANS),
    };
const lv_img_dsc_t  wd_img_9_spo2 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_17_MUSIC),
    };
const lv_img_dsc_t  wd_img_activity = {
    .header.always_zero = 0,
    .header.w = 86,
    .header.h = 86,
    .data_size = 7396 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY),
    };
const lv_img_dsc_t  wd_img_activity1_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY1_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_activity_comsume = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_COMSUME),
    };
const lv_img_dsc_t  wd_img_activity_comsume_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_COMSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_exercise = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_EXERCISE),
    };
const lv_img_dsc_t  wd_img_activity_exercise_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_step = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_STEP),
    };
const lv_img_dsc_t  wd_img_activity_step_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_STEP_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_activity_times = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_TIMES),
    };
const lv_img_dsc_t  wd_img_activity_times_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ACTIVITY_TIMES_SMALL),
    };
const lv_img_dsc_t  wd_img_battery = {
    .header.always_zero = 0,
    .header.w = 40,
    .header.h = 24,
    .data_size = 960 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY),
    };
const lv_img_dsc_t  wd_img_dark_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DARK_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_digital1_comsume_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL1_COMSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_exercise_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL1_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_heart_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL1_HEART_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_step_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL1_STEP_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_weather = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 38,
    .data_size = 1976 * 2,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL1_WEATHER),
    };
const lv_img_dsc_t  wd_img_digital2_consume_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL2_CONSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_exercise_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL2_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_heart_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL2_HEART_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_weather = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 38,
    .data_size = 1976 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL2_WEATHER),
    };
const lv_img_dsc_t  wd_img_flower_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_FLOWER_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_heartrate1 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE1),
    };
const lv_img_dsc_t  wd_img_heartrate2 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE2),
    };
const lv_img_dsc_t  wd_img_heartrate3 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE3),
    };
const lv_img_dsc_t  wd_img_heartrate4 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE4),
    };
const lv_img_dsc_t  wd_img_heartrate_bg = {
    .header.always_zero = 0,
    .header.w = 340,
    .header.h = 180,
    .data_size = 61200 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_BG),
    };
const lv_img_dsc_t  wd_img_heart_rate = {
    .header.always_zero = 0,
    .header.w = 44,
    .header.h = 44,
    .data_size = 1936 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_BG),
    };
const lv_img_dsc_t  wd_img_indicator = {
    .header.always_zero = 0,
    .header.w = 13,
    .header.h = 15,
    .data_size = 195 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_INDICATOR),
    };
const lv_img_dsc_t  wd_img_left_arrow = {
    .header.always_zero = 0,
    .header.w = 12,
    .header.h = 12,
    .data_size = 144 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LEFT_ARROW),
    };
const lv_img_dsc_t  wd_img_list_mode = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIST_MODE),
    };
const lv_img_dsc_t  wd_img_list_mode_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIST_MODE_SELECT),
    };
const lv_img_dsc_t  wd_img_live_flower_center = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 400 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_CENTER),
    };
const lv_img_dsc_t  wd_img_live_flower_hour = {
    .header.always_zero = 0,
    .header.w = 101,
    .header.h = 19,
    .data_size = 1919 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_HOUR),
    };
const lv_img_dsc_t  wd_img_live_flower_minute = {
    .header.always_zero = 0,
    .header.w = 170,
    .header.h = 16,
    .data_size = 2720 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_MINUTE),
    };
const lv_img_dsc_t  wd_img_live_flower_second = {
    .header.always_zero = 0,
    .header.w = 210,
    .header.h = 4,
    .data_size = 840 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_SECOND),
    };
const lv_img_dsc_t  wd_img_live_flower_watchface = {
    .header.always_zero = 0,
    .header.w = 452,
    .header.h = 452,
    .data_size = 204304 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_WATCHFACE),
    };
const lv_img_dsc_t  wd_img_live_flower_watchface1 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * 2,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_WATCHFACE1),
    };
const lv_img_dsc_t  wd_img_live_wallpaer_flower = {
    .header.always_zero = 0,
    .header.w = 260,
    .header.h = 260,
    .data_size = 67600 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAER_FLOWER),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_1 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_1),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_2 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_2),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_3 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_3),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_4 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_4),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_5 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_5),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_6 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_6),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_7 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_7),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_8 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAPER_MARS_8),
    };
const lv_img_dsc_t  wd_img_mapmarker = {
    .header.always_zero = 0,
    .header.w = 22,
    .header.h = 32,
    .data_size = 704 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MAPMARKER),
    };
const lv_img_dsc_t  wd_img_mars_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MARS_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_message = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE),
    };
const lv_img_dsc_t  wd_img_planet_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PLANET_VIEW),
    };
const lv_img_dsc_t  wd_img_planet_view_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PLANET_VIEW_SELECT),
    };
const lv_img_dsc_t  wd_img_radio_button_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_RADIO_BUTTON_SELECT),
    };
const lv_img_dsc_t  wd_img_radio_button_unselect = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_RADIO_BUTTON_UNSELECT),
    };
const lv_img_dsc_t  wd_img_stencil_logo_200p = {
    .header.always_zero = 0,
    .header.w = 200,
    .header.h = 200,
    .data_size = 40000 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STENCIL_LOGO_200P),
    };
const lv_img_dsc_t  wd_img_stencil_mask_200p = {
    .header.always_zero = 0,
    .header.w = 200,
    .header.h = 200,
    .data_size = 40000 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STENCIL_MASK_200P),
    };
const lv_img_dsc_t  wd_img_step2 = {
    .header.always_zero = 0,
    .header.w = 33,
    .header.h = 39,
    .data_size = 1287 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STEP2),
    };
const lv_img_dsc_t  wd_img_step3 = {
    .header.always_zero = 0,
    .header.w = 33,
    .header.h = 39,
    .data_size = 1287 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STEP3),
    };
const lv_img_dsc_t  wd_img_sudoku_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SUDOKU_VIEW),
    };
const lv_img_dsc_t  wd_img_sudoku_view_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SUDOKU_VIEW_SELECT),
    };
const lv_img_dsc_t  wd_img_sun2 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SUN2),
    };
const lv_img_dsc_t  wd_img_sun3 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SUN3),
    };
const lv_img_dsc_t  wd_img_watch1_center_point = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 23,
    .data_size = 529 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH1_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watch1_hour1 = {
    .header.always_zero = 0,
    .header.w = 121,
    .header.h = 15,
    .data_size = 1815 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH1_HOUR1),
    };
const lv_img_dsc_t  wd_img_watch1_minite1 = {
    .header.always_zero = 0,
    .header.w = 161,
    .header.h = 15,
    .data_size = 2415 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH1_MINITE1),
    };
const lv_img_dsc_t  wd_img_watch1_second1 = {
    .header.always_zero = 0,
    .header.w = 202,
    .header.h = 5,
    .data_size = 1010 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH1_SECOND1),
    };
const lv_img_dsc_t  wd_img_watch2_center_point = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 23,
    .data_size = 529 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH2_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watch2_hour1 = {
    .header.always_zero = 0,
    .header.w = 121,
    .header.h = 15,
    .data_size = 1815 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH2_HOUR1),
    };
const lv_img_dsc_t  wd_img_watch2_minite1 = {
    .header.always_zero = 0,
    .header.w = 161,
    .header.h = 15,
    .data_size = 2415 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH2_MINITE1),
    };
const lv_img_dsc_t  wd_img_watch2_second1 = {
    .header.always_zero = 0,
    .header.w = 202,
    .header.h = 5,
    .data_size = 1010 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH2_SECOND1),
    };
const lv_img_dsc_t  wd_img_watch3_center_point = {
    .header.always_zero = 0,
    .header.w = 14,
    .header.h = 14,
    .data_size = 196 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH3_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watchfaces2_1 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCHFACES2_1),
    };
const lv_img_dsc_t  wd_img_watch_face_2 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCH_FACE_2),
    };
const lv_img_dsc_t  wd_img_wechat = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_WECHAT),
    };
const lv_img_dsc_t  wd_img_alipay = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_ALIPAY),
    };
const lv_img_dsc_t  wd_img_netesae = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_NETEASE),
    };
const lv_img_dsc_t  wd_img_workhours = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_WORKOUTS),
    };
const lv_img_dsc_t  wd_img_phone = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_PHONE),
    };
const lv_img_dsc_t  wd_img_calander = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_CALANDER),
    };
const lv_img_dsc_t  wd_img_heartrate = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_HEARTRATE),
    };
const lv_img_dsc_t  wd_img_events = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1024 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MESSAGE_EVENTS),
    };
const lv_img_dsc_t  wd_img_white_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WHITE_THUMBNAIL),
    };

const lv_img_dsc_t  wd_img_nine_grid_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NINE_GRIDE_THUMBNAIL),
    };

const lv_img_dsc_t  wd_img_list3_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIST3_THUMBNAIL),
    };

const lv_img_dsc_t  wd_img_star_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STAR_THUMBNAIL),
    };

const lv_img_dsc_t  wd_img_list2_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIST2_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_list1_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGB565,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIST1_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_app_layout = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_1_APP_LAYOUT),
    };
    
const lv_img_dsc_t  wd_img_switch = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_2_SWITCH),
    };
const lv_img_dsc_t  wd_img_debug = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_4_DEBUG),
    };

 const lv_img_dsc_t  wd_img_language = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_3_LANGUAGE),
    };
 
 const lv_img_dsc_t  wd_img_menu = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_5_MENU),
    };
 
const lv_img_dsc_t  wd_img_settings = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_6_SETTINGS),
    };

const lv_img_dsc_t  wd_img_lightness = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 54,
    .data_size = 54 * 54 * 4,
#if GR552X_GPU_IMG_SUPPORT > 0u
    .header.cf = LV_IMG_CF_GDX_RGBA8888,
#else
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
#endif
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_7_LIGHTNESS),
    };
