/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_periph_setup.h"
#include "scatter_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "grx_sys.h"
#include "bsp_tp.h"
#include "app_log.h"
#include "app_io.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "app_graphics_mem.h"
#include "platform_sdk.h"
#include "lv_wms.h"
#include "lv_wms_surface_flinger.h"
#include "lv_layout_manager.h"
#include "app_rtc.h"
#include "disp_driver.h"
#include "app_key.h"

/*
 * MACRO DEFINITIONS
 *****************************************************************************************
 */
#define LV_TIMEROUT_THRES        (10 * 1000)
#define LV_TASK_STACK_SIZE       (1024 * 4)//unit : word
#define LV_MEM_USED_MONITOR      (1)

/*
 * GLOBAL PMU FUNCTION DECLARATIONS
 *****************************************************************************************
 */
typedef struct
{
    uint16_t dcdc_mv;     /**< dcdc vout mv. */
    uint16_t dcore_mv;    /**< dcore vout mv. */
    uint16_t dcdc_vout;   /**< dcdc vout code. */
    uint16_t dcore_vout;  /**< dcore vout code. */
    bool dcore_bypass_en; /**< dcore bypass enable. */
}dcdc_dcore_info_t;
extern void sys_pmu_dcdc_set(uint32_t dcdc_96m_mv, uint32_t dcdc_64m_mv);
extern void sys_pmu_digcore_set(uint32_t dcore_96m_mv, uint32_t dcore_64m_mv);
extern dcdc_dcore_info_t g_dcdc_dcore_set_for_gpu;
extern void pmu_dcdc_dcore_info_record(dcdc_dcore_info_t* p_dcdc_dcore);

/*
 * GLOBAL GUI OBJECT DECLARATIONS
 *****************************************************************************************
 */
extern lv_indev_t * indev_touchpad;
extern void lv_home_layout_init(void);
extern void touchpad_indev_cache(void);

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t tp_task_handle;
static TaskHandle_t gui_task_handle;
static bool lv_env_is_inited = false;
static SemaphoreHandle_t s_gui_refresh_sem;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void lvgl_env_init(void)
{
    sys_pmu_dcdc_set(1120 /* 96MHz */, 1120 /* 64MHz */);
    sys_pmu_digcore_set(1120/* 96MHz */, 1120 /* 64MHz */);
    pmu_dcdc_dcore_info_record(&g_dcdc_dcore_set_for_gpu);

    app_graphics_ospi_params_t params = PSRAM_INIT_PARAMS_Default;
    app_graphics_ospi_init(&params);

    mem_pwr_mgmt_mode_set(MEM_POWER_FULL_MODE);
    app_graphics_mem_init((void*)GFX_MEM_BASE, GFX_MEM_SIZE);

    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    lv_env_is_inited = true;
    lv_home_layout_init();
    app_rtc_init(NULL);
    s_gui_refresh_sem = xSemaphoreCreateBinary();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void gui_schedule_task(void *p_arg)
{
    lvgl_env_init();
    uint32_t delayTime = 0;
    while(1){
        delayTime = lv_task_handler();
        xSemaphoreTake(s_gui_refresh_sem, delayTime);
    }
}

void lv_indev_read_task(void * args)
{
    while(1){
        if(lv_env_is_inited){
            touchpad_indev_cache();
        }
        xSemaphoreGive(s_gui_refresh_sem);
        if(lv_wms_is_in_busy_state()){
            if(LV_TRANS_EFFECT_CUBE == lv_wms_transit_effect_get()){
               vTaskDelay(30); // cube effect takes around 30ms
            }else{
               vTaskDelay(25); // other effect takes around 25ms
            }
        }else{
            vTaskDelay(LV_INDEV_DEF_READ_PERIOD);
        }
    }
}

void lvgl_mem_used_dump(const char* func, int line){
#if LV_MEM_USED_MONITOR
    static uint32_t _lv_mem_used_size = 0;
    static uint32_t _lv_mem_max_size = 0;
    _lv_mem_used_size = lv_mem_used_size_get();
    _lv_mem_max_size = LV_MAX(_lv_mem_used_size, _lv_mem_max_size);
    printf("LVGL View: %s, Line: %d, used size: %d, max size: %d\n", 
    func, line, _lv_mem_used_size, _lv_mem_max_size);
#endif
}

/**
 *****************************************************************************************
 * @brief To create two task, the one is ble-schedule, another is watcher task
 *****************************************************************************************
 */
void lv_user_task_create(void)
{
    xTaskCreate(lv_indev_read_task, "indev_task", 512, NULL, configMAX_PRIORITIES - 1, &tp_task_handle);
    xTaskCreate(gui_schedule_task, "gui_task", LV_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, &gui_task_handle);
}
