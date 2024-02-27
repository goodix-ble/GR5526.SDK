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
#include "grx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "watcher.h"
#include "custom_config.h"
#include "patch.h"
#include "grx_hal.h"
#include "app_log.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
TaskHandle_t app_task_handle;

#define APP_TASK_STACK_SIZE (2*1024)//unit : word
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief To create application task
 *****************************************************************************************
 */
extern int ospi_psram_access_test(void);        /* Access Test */
extern int ospi_psram_pressure_test(void);      /* Pressure Test */
extern int ospi_psram_half_sleep_test(void);

static void ospi_application_task(void *p_arg)
{
//    ospi_psram_access_test();
//    ospi_psram_pressure_test();
    ospi_psram_half_sleep_test();

    while(1){
        delay_ms(1000);
        printf("sleep...\r\n");
    };
}

/**
 *****************************************************************************************
 * @brief To create qspi task
 *****************************************************************************************
 */

static void vStartTasks(void *arg)
{
    xTaskCreate(ospi_application_task, "graphics_task", APP_TASK_STACK_SIZE, NULL, 0, &app_task_handle);
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

    printf("\r\n");
    printf("****************************************************&*\r\n");
    printf("*            OSPI PSRAM ACCESS Example               *\r\n");
    printf("*                                                    *\r\n");
    printf("******************************************************\r\n");

    xTaskCreate(vStartTasks, "create_task", 512, NULL, 0, NULL);    /*< create some demo tasks via freertos */
    vTaskStartScheduler();                                          /*< freertos run all tasks*/
    while(1);                                                       /*< Never perform here */
}
