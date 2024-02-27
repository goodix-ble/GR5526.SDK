/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

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

/**
 *****************************************************************************************
 *      DECLARATIONS
  *****************************************************************************************
**/
void lv_user_task_create(void);

/**
 *****************************************************************************************
 * @brief To create two task, the one is ble-schedule, another is watcher task
 *****************************************************************************************
 */
static void vStartTasks(void *arg)
{
    lv_user_task_create();
    vTaskDelete(NULL);
}


/**
 *****************************************************************************************
 * @brief main function
 *****************************************************************************************
 */
int main(void)
{
    SetSerialClock(SERIAL_N96M_CLK);
    app_periph_init();                                              /*<init user periph .*/

    printf("Graphics LVGL Watch Demo Start\r\n");
    xTaskCreate(vStartTasks, "create_task", 1024, NULL, 0, NULL);   /*< create some demo tasks via freertos */
    vTaskStartScheduler();                                          /*< freertos run all tasks*/
    for (;;);                                                       /*< Never perform here */
}

void vApplicationStackOverflowHook (TaskHandle_t xTask, char *pcTaskName) {
    printf(">>>> FReeRTOS Task %s Overflow  !!!\r\n", pcTaskName);
}

void vApplicationMallocFailedHook () {
    printf(">>>> FReeRTOS Malloc FAILED !!!\r\n");
}
