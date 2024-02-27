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
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "grx_hal.h"
#include "app_aon_wdt.h"
#include "board_SK.h"
#include "grx_sys.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#define APP_AON_WDT_TEST_COUNT  10
app_aon_wdt_params_t s_params;
/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_aon_wdt_evt_handler(void)
{
    printf("app_aon_wdt_evt\r\n");
}

void app_aon_wdt(void)
{
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    s_params.init.counter = 32768 * 2 - 1;
    s_params.init.alarm_counter = 0x1F;
#else
    s_params.init.counter = 2000;
    s_params.init.alarm_counter = 1000;
#endif

    app_aon_wdt_init(&s_params, app_aon_wdt_evt_handler);

    for (uint32_t i = 0; i < APP_AON_WDT_TEST_COUNT; i++)
    {
        delay_ms(300);
        app_aon_wdt_refresh();
        printf("\r\n %dth feed dog.\r\n", i);
    }

    printf("\r\nSystem will reset.\r\n");
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("*******************************************************\r\n");
    printf("*                AON_WDT_RESET example.               *\r\n");
    printf("*                                                     *\r\n");
    printf("* This sample will show the AON_WDT reset system.     *\r\n");
    printf("* We will freed dog 10 times every 300ms.             *\r\n");
    printf("* there will be an interruption if the AON_WDT is not *\r\n");
    printf("* refreshed within 1s.the system will reset if the    *\r\n");
    printf("* AON_WDT is not refreshed within 2s.                 *\r\n");
    printf("*******************************************************\r\n");

    app_aon_wdt();

    while(1);
}
