#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
const lv_img_dsc_t  wd_img_10_upgrade= {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_10_COMPASS),
    };
const lv_img_dsc_t  wd_img_11_About = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_11_ABOUT),
    };
const lv_img_dsc_t  wd_img_12_taobao = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_12_TAOBAO),
    };
const lv_img_dsc_t  wd_img_13_entertainment = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_13_ENTERTAINMENT),
    };
const lv_img_dsc_t  wd_img_14_phonebook = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_14_PHONEBOOK),
    };
const lv_img_dsc_t  wd_img_15_Googlemap = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_15_GOOGLEMAP),
    };
const lv_img_dsc_t  wd_img_16_baidu = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_16_BAIDU),
    };
const lv_img_dsc_t  wd_img_17_iqiyi = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_17_IQIYI),
    };
const lv_img_dsc_t  wd_img_18_netease = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_18_NETEASE),
    };
const lv_img_dsc_t  wd_img_19_wallet = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_19_WALLET),
    };
const lv_img_dsc_t  wd_img_1_App_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_1_APP_VIEW),
    };
const lv_img_dsc_t  wd_img_1_setting = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_1_SETTING),
    };
const lv_img_dsc_t  wd_img_20_music = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_20_MUSIC),
    };
const lv_img_dsc_t  wd_img_21_calendar = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_21_CALENDAR),
    };
const lv_img_dsc_t  wd_img_22_Chrome = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_22_CHROME),
    };
const lv_img_dsc_t  wd_img_23_twitter = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_23_TWITTER),
    };
const lv_img_dsc_t  wd_img_24_WhatsApp = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_24_WHATSAPP),
    };
const lv_img_dsc_t  wd_img_25_cmbc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_25_CMBC),
    };
const lv_img_dsc_t  wd_img_26_online_class = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_26_ONLINE_CLASS),
    };
const lv_img_dsc_t  wd_img_27_message = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_27_MESSAGE),
    };
const lv_img_dsc_t  wd_img_28_Google_meetting = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_28_GOOGLE_MEETTING),
    };
const lv_img_dsc_t  wd_img_29_facebook = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_29_FACEBOOK),
    };
const lv_img_dsc_t  wd_img_2_alipay = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_2_ALIPAY),
    };
const lv_img_dsc_t  wd_img_2_function_key = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_2_FUNCTION_KEY),
    };
const lv_img_dsc_t  wd_img_30_douban = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_30_DOUBAN),
    };
const lv_img_dsc_t  wd_img_31_QQ_space = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_31_QQ_SPACE),
    };
const lv_img_dsc_t  wd_img_32_google_plus = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_32_GOOGLE_PLUS),
    };
const lv_img_dsc_t  wd_img_33_weather = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_33_WEATHER),
    };
const lv_img_dsc_t  wd_img_34_google_play = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_34_GOOGLE_PLAY),
    };
const lv_img_dsc_t  wd_img_35_application_store = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_35_APPLICATION_STORE),
    };
const lv_img_dsc_t  wd_img_36_icbc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_36_ICBC),
    };
const lv_img_dsc_t  wd_img_37_QQ_music = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_37_QQ_MUSIC),
    };
const lv_img_dsc_t  wd_img_38_file_manage = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_38_FILE_MANAGE),
    };
const lv_img_dsc_t  wd_img_39_QQ = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_39_QQ),
    };
const lv_img_dsc_t  wd_img_3_clock_setting = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_3_CLOCK_SETTING),
    };
const lv_img_dsc_t  wd_img_3_wechat = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_3_WECHAT),
    };
const lv_img_dsc_t  wd_img_40_yahoo = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_40_YAHOO),
    };
const lv_img_dsc_t  wd_img_4_favorite = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_4_FAVORITE),
    };
const lv_img_dsc_t  wd_img_4_heart = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_4_HEART),
    };
const lv_img_dsc_t  wd_img_5_dianping = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_5_DIANPING),
    };
const lv_img_dsc_t  wd_img_5_notice_manage = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_5_NOTICE_MANAGE),
    };
const lv_img_dsc_t  wd_img_6_camera = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_6_CAMERA),
    };
const lv_img_dsc_t  wd_img_6_system_update = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_6_SYSTEM_UPDATE),
    };
const lv_img_dsc_t  wd_img_7_general_setting = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_7_GENERAL_SETTING),
    };
const lv_img_dsc_t  wd_img_7_tiktok = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_7_TIKTOK),
    };
const lv_img_dsc_t  wd_img_8_activity = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_8_ACTIVITY),
    };
const lv_img_dsc_t  wd_img_8_auxiliary_means = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_8_AUXILIARY_MEANS),
    };
const lv_img_dsc_t  wd_img_9_spo2 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_9_SPO2),
    };
const lv_img_dsc_t  wd_img_activity = {
    .header.always_zero = 0,
    .header.w = 86,
    .header.h = 86,
    .data_size = 7396 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY),
    };
const lv_img_dsc_t  wd_img_activity1_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY1_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_activity_comsume = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_COMSUME),
    };
const lv_img_dsc_t  wd_img_activity_comsume_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_COMSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_exercise = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_EXERCISE),
    };
const lv_img_dsc_t  wd_img_activity_exercise_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_step = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_STEP),
    };
const lv_img_dsc_t  wd_img_activity_step_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_STEP_SMALL),
    };
const lv_img_dsc_t  wd_img_activity_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_activity_times = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 1296 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_TIMES),
    };
const lv_img_dsc_t  wd_img_activity_times_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_ACTIVITY_TIMES_SMALL),
    };
const lv_img_dsc_t  wd_img_battery = {
    .header.always_zero = 0,
    .header.w = 40,
    .header.h = 24,
    .data_size = 960 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_BATTERY),
    };
const lv_img_dsc_t  wd_img_dark_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DARK_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_digital1_comsume_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL1_COMSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_exercise_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL1_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_heart_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL1_HEART_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_step_small = {
    .header.always_zero = 0,
    .header.w = 10,
    .header.h = 10,
    .data_size = 100 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL1_STEP_SMALL),
    };
const lv_img_dsc_t  wd_img_digital1_weather = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 38,
    .data_size = 1976 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL1_WEATHER),
    };
const lv_img_dsc_t  wd_img_digital2_consume_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL2_CONSUME_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_exercise_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL2_EXERCISE_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_heart_small = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 576 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL2_HEART_SMALL),
    };
const lv_img_dsc_t  wd_img_digital2_weather = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 38,
    .data_size = 1976 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_DIGITAL2_WEATHER),
    };
const lv_img_dsc_t  wd_img_flower_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_FLOWER_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_heartrate1 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEARTRATE1),
    };
const lv_img_dsc_t  wd_img_heartrate2 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEARTRATE2),
    };
const lv_img_dsc_t  wd_img_heartrate3 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEARTRATE3),
    };
const lv_img_dsc_t  wd_img_heartrate4 = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 80,
    .data_size = 6400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEARTRATE4),
    };
const lv_img_dsc_t  wd_img_heartrate_bg = {
    .header.always_zero = 0,
    .header.w = 340,
    .header.h = 180,
    .data_size = 61200 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEARTRATE_BG),
    };
const lv_img_dsc_t  wd_img_heart_rate = {
    .header.always_zero = 0,
    .header.w = 44,
    .header.h = 44,
    .data_size = 1936 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_HEART_RATE),
    };
const lv_img_dsc_t  wd_img_indicator = {
    .header.always_zero = 0,
    .header.w = 13,
    .header.h = 15,
    .data_size = 195 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_INDICATOR),
    };
const lv_img_dsc_t  wd_img_left_arrow = {
    .header.always_zero = 0,
    .header.w = 12,
    .header.h = 12,
    .data_size = 144 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LEFT_ARROW),
    };
const lv_img_dsc_t  wd_img_list_mode = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIST_MODE),
    };
const lv_img_dsc_t  wd_img_list_mode_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIST_MODE_SELECT),
    };
const lv_img_dsc_t  wd_img_live_flower_center = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 400 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_CENTER),
    };
const lv_img_dsc_t  wd_img_live_flower_hour = {
    .header.always_zero = 0,
    .header.w = 101,
    .header.h = 19,
    .data_size = 1919 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_HOUR),
    };
const lv_img_dsc_t  wd_img_live_flower_minute = {
    .header.always_zero = 0,
    .header.w = 170,
    .header.h = 16,
    .data_size = 2720 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_MINUTE),
    };
const lv_img_dsc_t  wd_img_live_flower_second = {
    .header.always_zero = 0,
    .header.w = 210,
    .header.h = 4,
    .data_size = 840 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_SECOND),
    };
const lv_img_dsc_t  wd_img_live_flower_watchface = {
    .header.always_zero = 0,
    .header.w = 452,
    .header.h = 452,
    .data_size = 204304 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_WATCHFACE),
    };
const lv_img_dsc_t  wd_img_live_flower_watchface1 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_FLOWER_WATCHFACE1),
    };
const lv_img_dsc_t  wd_img_live_wallpaer_flower = {
    .header.always_zero = 0,
    .header.w = 260,
    .header.h = 260,
    .data_size = 67600 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAER_FLOWER),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_1 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_1),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_2 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_2),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_3 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_3),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_4 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_4),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_5 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_5),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_6 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_6),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_7 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_7),
    };
const lv_img_dsc_t  wd_img_live_wallpaper_mars_8 = {
    .header.always_zero = 0,
    .header.w = 150,
    .header.h = 150,
    .data_size = 22500 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_LIVE_WALLPAPER_MARS_8),
    };
const lv_img_dsc_t  wd_img_mapmarker = {
    .header.always_zero = 0,
    .header.w = 22,
    .header.h = 32,
    .data_size = 704 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_MAPMARKER),
    };
const lv_img_dsc_t  wd_img_mars_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_MARS_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_message = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_MESSAGE),
    };
const lv_img_dsc_t  wd_img_planet_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_PLANET_VIEW),
    };
const lv_img_dsc_t  wd_img_planet_view_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_PLANET_VIEW_SELECT),
    };
const lv_img_dsc_t  wd_img_radio_button_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_RADIO_BUTTON_SELECT),
    };
const lv_img_dsc_t  wd_img_radio_button_unselect = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_RADIO_BUTTON_UNSELECT),
    };
const lv_img_dsc_t  wd_img_stencil_logo_200p = {
    .header.always_zero = 0,
    .header.w = 200,
    .header.h = 200,
    .data_size = 40000 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_STENCIL_LOGO_200P),
    };
const lv_img_dsc_t  wd_img_stencil_mask_200p = {
    .header.always_zero = 0,
    .header.w = 200,
    .header.h = 200,
    .data_size = 40000 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_STENCIL_MASK_200P),
    };
const lv_img_dsc_t  wd_img_step2 = {
    .header.always_zero = 0,
    .header.w = 33,
    .header.h = 39,
    .data_size = 1287 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_STEP2),
    };
const lv_img_dsc_t  wd_img_step3 = {
    .header.always_zero = 0,
    .header.w = 33,
    .header.h = 39,
    .data_size = 1287 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_STEP3),
    };
const lv_img_dsc_t  wd_img_sudoku_view = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_SUDOKU_VIEW),
    };
const lv_img_dsc_t  wd_img_sudoku_view_select = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_SUDOKU_VIEW_SELECT),
    };
const lv_img_dsc_t  wd_img_sun2 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_SUN2),
    };
const lv_img_dsc_t  wd_img_sun3 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_SUN3),
    };
const lv_img_dsc_t  wd_img_watch1_center_point = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 23,
    .data_size = 529 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH1_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watch1_hour1 = {
    .header.always_zero = 0,
    .header.w = 121,
    .header.h = 15,
    .data_size = 1815 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH1_HOUR1),
    };
const lv_img_dsc_t  wd_img_watch1_minite1 = {
    .header.always_zero = 0,
    .header.w = 161,
    .header.h = 15,
    .data_size = 2415 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH1_MINITE1),
    };
const lv_img_dsc_t  wd_img_watch1_second1 = {
    .header.always_zero = 0,
    .header.w = 202,
    .header.h = 5,
    .data_size = 1010 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH1_SECOND1),
    };
const lv_img_dsc_t  wd_img_watch2_center_point = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 23,
    .data_size = 529 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH2_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watch2_hour1 = {
    .header.always_zero = 0,
    .header.w = 121,
    .header.h = 15,
    .data_size = 1815 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH2_HOUR1),
    };
const lv_img_dsc_t  wd_img_watch2_minite1 = {
    .header.always_zero = 0,
    .header.w = 161,
    .header.h = 15,
    .data_size = 2415 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH2_MINITE1),
    };
const lv_img_dsc_t  wd_img_watch2_second1 = {
    .header.always_zero = 0,
    .header.w = 202,
    .header.h = 5,
    .data_size = 1010 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH2_SECOND1),
    };
const lv_img_dsc_t  wd_img_watch3_center_point = {
    .header.always_zero = 0,
    .header.w = 14,
    .header.h = 14,
    .data_size = 196 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH3_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_watchfaces2_1 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCHFACES2_1),
    };
const lv_img_dsc_t  wd_img_watch_face_2 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 206116 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WATCH_FACE_2),
    };
const lv_img_dsc_t  wd_img_wechat = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 32,
    .data_size = 1024 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WECHAT),
    };
const lv_img_dsc_t  wd_img_white_thumbnail = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 270,
    .data_size = 72900 * LV_COLOR_SIZE / 8, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_BASE + ADDR_WHITE_THUMBNAIL),
    };
