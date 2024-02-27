#include "stdio.h"
#include "lv_wms.h"
#include "layout_scene_manage.h"

#if WMS_VERSION == WMS_VERSION_v2

/*
 * DEFINITIONS for Window MAP
 *****************************************************************************************
 */

extern lv_wms_window_t      g_card_watch_win;
extern lv_wms_window_t      g_card_message_win;
extern lv_wms_window_t      g_card_status_win;
extern lv_wms_window_t      g_card_list_entry_win;
extern lv_wms_window_t      g_card_day_activity_win;
extern lv_wms_window_t      g_card_heartrate_win;
extern lv_wms_window_t      g_card_spo2_win;
extern lv_wms_window_t      g_card_stress_win;
extern lv_wms_window_t      g_card_sleep_win;
extern lv_wms_window_t      g_card_music_win;
extern lv_wms_window_t      g_card_nfc_win;
extern lv_wms_window_t      g_list_menu_win;
extern lv_wms_window_t      g_sports_win;



const lv_wms_window_map_t g_win_map_table[] = {
    {WMS_WIN_CARD_WATCHFACE_ID,     &g_card_watch_win   },
    {WMS_WIN_CARD_MESSAGE_ID,       &g_card_message_win },
    {WMS_WIN_CARD_STATUS_ID,        &g_card_status_win  },
    {WMS_WIN_CARD_SPO2_ID,          &g_card_spo2_win    },
    {WMS_WIN_CARD_SLEEP_ID,         &g_card_sleep_win   },
    {WMS_WIN_CARD_STRESS_ID,        &g_card_stress_win  },
    {WMS_WIN_CARD_LIST_ENTRY_ID,    &g_card_list_entry_win      },
    {WMS_WIN_CARD_DAY_ACTIVITY_ID,  &g_card_day_activity_win    },
    {WMS_WIN_CARD_HEARTRATE_ID,     &g_card_heartrate_win       },
    {WMS_WIN_CARD_MUSIC_ID,         &g_card_music_win   },
    {WMS_WIN_CARD_NFC_ID,           &g_card_nfc_win     },

    {WMS_WIN_LIST_MENU_ID,          &g_list_menu_win    },
    {WMS_WIN_SPORTS_ID,             &g_sports_win       },
};

#define LV_WMS_WINDOW_COUNT     (sizeof(g_win_map_table)/sizeof(lv_wms_window_map_t))



/*
 * DEFINITIONS for Scenes MAP
 *****************************************************************************************
 */


/*
 * SCENE Define: Main
 **************************
 */

#define MAIN_SCENE_X_ELEMS      9
#define MAIN_SCENE_Y_ELEMS      3

uint32_t s_main_scene[MAIN_SCENE_Y_ELEMS][MAIN_SCENE_X_ELEMS] = {
    {0,                          WMS_WIN_CARD_MESSAGE_ID,   0,                            0,                         0,                    0,                      0,                     0,                     0                  },
    {WMS_WIN_CARD_LIST_ENTRY_ID, WMS_WIN_CARD_WATCHFACE_ID, WMS_WIN_CARD_DAY_ACTIVITY_ID, WMS_WIN_CARD_HEARTRATE_ID, WMS_WIN_CARD_SPO2_ID, WMS_WIN_CARD_STRESS_ID, WMS_WIN_CARD_SLEEP_ID, WMS_WIN_CARD_MUSIC_ID, WMS_WIN_CARD_NFC_ID},
    {0,                          WMS_WIN_CARD_STATUS_ID,    0,                            0,                         0,                    0,                      0,                     0,                     0                  },
};



/*
 * SCENE Define: Second
 **************************
 */
#define SECOND_SCENE_X_ELEMS      1
#define SECOND_SCENE_Y_ELEMS      1

uint32_t s_second_scene[SECOND_SCENE_Y_ELEMS][SECOND_SCENE_X_ELEMS] = {
    {WMS_WIN_LIST_MENU_ID},
};


/*
 * SCENE Define: Third
 **************************
 */

#define THIRD_SCENE_X_ELEMS      3
#define THIRD_SCENE_Y_ELEMS      3

uint32_t s_third_scene[THIRD_SCENE_Y_ELEMS][THIRD_SCENE_X_ELEMS] = {
    {0,                     WMS_WIN_CARD_STRESS_ID,        0,                   },
    {WMS_WIN_CARD_SPO2_ID,  WMS_WIN_SPORTS_ID,             WMS_WIN_CARD_WATCHFACE_ID},
    {0,                     WMS_WIN_CARD_SLEEP_ID,         0,                   },
};


/*
 * SCENE Define: Scene MAP
 *****************************************************************************************
 */

const lv_wms_scene_t s_scene_map[] = {
    {WMS_SCENE_MAIN_ID,     WMS_WIN_CARD_WATCHFACE_ID,   MAIN_SCENE_X_ELEMS,     MAIN_SCENE_Y_ELEMS,     (uint32_t*)s_main_scene},
    {WMS_SCENE_SECOND_ID,   WMS_WIN_LIST_MENU_ID,        SECOND_SCENE_X_ELEMS,   SECOND_SCENE_Y_ELEMS,   (uint32_t*)s_second_scene},
    {WMS_SCENE_THIRD_ID,    WMS_WIN_CARD_SPO2_ID,        THIRD_SCENE_X_ELEMS,    THIRD_SCENE_Y_ELEMS,    (uint32_t*)s_third_scene},
};


#define LV_WMS_SCENE_COUNT     (sizeof(s_scene_map)/sizeof(lv_wms_scene_t))



/*
 * DEFINITIONS for Static & APIs
 *****************************************************************************************
 */
#if 0
void find_test() {
    lv_wms_win_neighbor_id_t n_id;

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_MESSAGE_CARD_ID);
    printf("MESSAGE [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_SLEEP_CARD_ID);
    printf("SLEEP [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_WATCHFACE_ID);
    printf("WATCH [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_STATUS_CARD_ID);
    printf("STATUS [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_SPO2_CARD_ID);
    printf("SPO2 [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );

    n_id = lv_wms_scene_find_all_neighbor_window_id(WMS_SCENE_MAIN_ID, WIN_STRESS_CARD_ID);
    printf("STRESS [%d, %d, %d, %d] \r\n", n_id.win_up_id, n_id.win_down_id, n_id.win_left_id, n_id.win_right_id );
}
#endif


void lv_wms_scene_init(void) {

    lv_wms_scene_config_t _config = {
        .p_win_map_table        = (lv_wms_window_map_t*)&g_win_map_table[0],
        .p_scene_map_table      = (lv_wms_scene_t*)s_scene_map,
        .win_amount             = LV_WMS_WINDOW_COUNT,
        .scene_amount           = LV_WMS_SCENE_COUNT,
        .startup_scene_id       = WMS_SCENE_MAIN_ID,
        .startup_win_id         = WMS_WIN_CARD_WATCHFACE_ID,
    };

    lv_wms_scene_startup(&_config);
}

#endif /* WMS_VERSION == WMS_VERSION_v2 */
