#include "stdint.h"
#include "grx_sys.h"

#include "osal.h"

#include "lvgl.h"
#include "app_sys_manager.h"
#include "app_ui_manager.h"
#include "app_key_core.h"
#include "lv_layout_router.h"

#define GUI_EVENT_TASK_PRIORITY (7)

typedef struct {
    uint16_t    evt_id;
    void *      data;
} lv_wms_gui_evt_t;

/* defined by user */
typedef enum {
    WMS_GUI_EVT_NONE,
    WMS_GUI_EVT_KEY0_PRESSED,
    WMS_GUI_EVT_KEY0_LONG_PRESSED,
    WMS_GUI_EVT_KEY1_PRESSED,
    WMS_GUI_EVT_KEY1_LONG_PRESSED,
    WMS_GUI_EVT_KEY1_CONTINUE_PRESS,
    WMS_GUI_EVT_GESTURE_HORI,
} lv_wms_gui_evt_e;

static osal_queue_handle_t s_gui_evt_queue = NULL;

static void _lv_gui_goto_applist(void* user_data) {
    app_ui_mgr_goto_applist();
}

static void _lv_gui_goto_root(void* user_data) {
    app_ui_mgr_goto_root();
}

static lv_img_dsc_t*    p_snapshot = NULL;
extern void             lv_scr_load_anim_rotate(lv_obj_t * cur, uint32_t src, uint32_t time, uint32_t delay, uint32_t start, uint32_t end, uint32_t anim, void * data);
extern lv_img_dsc_t *   lv_snapshot_take_ext(lv_obj_t *obj, lv_img_cf_t cf);

static uint32_t s_anim  = 0; 
bool volatile is_gui_win_rotated = false;
static void _lv_gui_rotate_win(void * data) {

    lv_obj_t * p_cur_obj = lv_layout_router_get_active_obj();

    printf("ACTIVE SCN: 0x%08x \r\n", (uint32_t) p_cur_obj);
    p_snapshot = lv_snapshot_take_ext(p_cur_obj, LV_IMG_CF_GDX_RGB565);

    if(!is_gui_win_rotated) {
        printf("Rotate FROM 0 -> 180 \r\n");
        lv_scr_load_anim_rotate(p_cur_obj, (uint32_t)p_snapshot->data, 500, 0, 0, 1800, s_anim++ % 5, p_snapshot);
        is_gui_win_rotated = true;
    } else {
        printf("Rotate FROM 180 -> 360 \r\n");
        lv_scr_load_anim_rotate(p_cur_obj, (uint32_t)p_snapshot->data, 500, 0, 1800, 3600, s_anim++ % 5, p_snapshot);
        is_gui_win_rotated = false;
    }
}

static void _lv_gui_goto_power_off(void* user_data) {
    lv_layout_router_goto_isolate_win(lv_layout_router_get_active_obj(), WMS_WID_POWER_OFF, WMS_DEFAULT_GOTO_EFFECT_POS, LV_WMS_TILEVIEW_EFFECT_SPIN);
}

static void app_event_task(void *p_arg) {
    osal_base_type_t xRet = OSAL_SUCCESS;

    lv_wms_gui_evt_t evt;

    while(1) {
        xRet = osal_queue_receive(s_gui_evt_queue, &evt, OSAL_MAX_DELAY);

        if (OSAL_SUCCESS == xRet) {

            switch(evt.evt_id) {
                case WMS_GUI_EVT_KEY0_PRESSED:
                {
                    if(app_ui_mgr_is_at_root()) {
                        printf("GOTO AppList\r\n");
                        lv_async_call(_lv_gui_goto_applist, NULL);
                    } else {
                        printf("GOTO Root\r\n");
                        lv_async_call(_lv_gui_goto_root, NULL);
                    }
                }
                break;

                case WMS_GUI_EVT_KEY0_LONG_PRESSED:
                {
                    if(app_ui_mgr_is_at_root()) {
                        printf("GOTO PowerOff\r\n");
                        lv_async_call(_lv_gui_goto_power_off, NULL);
                    }
                }
                break;

                case WMS_GUI_EVT_KEY1_PRESSED:
                case WMS_GUI_EVT_KEY1_CONTINUE_PRESS:
                {
                    lv_async_call(_lv_gui_rotate_win, NULL);
                }
                break;

                case WMS_GUI_EVT_KEY1_LONG_PRESSED:
                {

                }
                break;

                default:
                {
                    printf("Msg %02x not handled \r\n", evt.evt_id);
                }
                break;
            }

        } else {
            printf("Error PATH!\r\n");
        }
    }
}

void lv_gui_evt_task_startup(void) {
    osal_queue_create(&s_gui_evt_queue, 10, sizeof(lv_wms_gui_evt_t));
    osal_task_create("task_evt", app_event_task, TASK_SIZE_APP_EVENT, TASK_PRIO_APP_EVENT, &g_task_handle.gui_evt_handle);

    return;
}

bool lv_gui_evt_send(uint16_t evt_id, void * data) {

    if(!s_gui_evt_queue) {
        return false;
    }

    lv_wms_gui_evt_t gui_evt = {
        .evt_id   = evt_id,
        .data     = data,
    };

    osal_base_type_t xRet = osal_queue_send(s_gui_evt_queue, &gui_evt, OSAL_MAX_DELAY);
    return (xRet == OSAL_SUCCESS) ? true : false;
}

extern void _key_drv_irq_notify(void);
/*
 * override the __weak function in board_SK.c
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    if((key_id == 0) && (APP_KEY_SINGLE_CLICK == key_click_type)) {
        lv_gui_evt_send(WMS_GUI_EVT_KEY0_PRESSED, NULL);
    } else if((key_id == 0) && (APP_KEY_LONG_CLICK == key_click_type)) {
        lv_gui_evt_send(WMS_GUI_EVT_KEY0_LONG_PRESSED, NULL);
    } else if((key_id == 1) && (APP_KEY_SINGLE_CLICK == key_click_type)) {
        lv_gui_evt_send(WMS_GUI_EVT_KEY1_PRESSED, NULL);
    } else if((key_id == 1) && (APP_KEY_LONG_CLICK == key_click_type)) {
        lv_gui_evt_send(WMS_GUI_EVT_KEY1_LONG_PRESSED, NULL);
    }  else if((key_id == 1) && (APP_KEY_CONTINUE_CLICK == key_click_type)) {
        lv_gui_evt_send(WMS_GUI_EVT_KEY1_CONTINUE_PRESS, NULL);
    }

    // Any Key can wake device from sleep state
    _key_drv_irq_notify();

    return;
}
