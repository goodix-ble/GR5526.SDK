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
#include "FreeRTOS.h"
#include "task.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_TASK_STACK_SIZE (2*1024)//unit : word

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
TaskHandle_t app_task_handle;

/**
 *****************************************************************************************
 * @brief To create application task
 *****************************************************************************************
 */
extern void dspi_dev_application_task(void *p_arg);

/**
 *****************************************************************************************
 * @brief To create qspi task
 *****************************************************************************************
 */

static void vStartTasks(void *arg)
{
    xTaskCreate(dspi_dev_application_task, "graphics_task", APP_TASK_STACK_SIZE, NULL, 0, &app_task_handle);
    vTaskDelete(NULL);
}

/**
 *****************************************************************************************
 * @brief main function
 *****************************************************************************************
 */
int main(void)
{
    app_periph_init();                                              /*<init user periph .*/

    printf("\r\n");
    printf("****************************************************&*\r\n");
    printf("*            DSPI: Device Access Demo                *\r\n");
    printf("*            DSPI: <----------> 454p LCD             *\r\n");
    printf("******************************************************\r\n");

    xTaskCreate(vStartTasks, "create_task", 512, NULL, 0, NULL);    /*< create some demo tasks via freertos */
    vTaskStartScheduler();                                          /*< freertos run all tasks*/
    while(1);                                                       /*< Never perform here */
}
