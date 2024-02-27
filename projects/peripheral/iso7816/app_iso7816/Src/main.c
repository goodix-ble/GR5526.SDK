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
#include "app_io.h"
#include "app_iso7816.h"
#include "board_SK.h"
#include "gr55xx_sim_card.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/*******ISO7816 DRIVER IO CONFIG*******************/
#define ISO7816_IO_SEL_GPIOA                0
#define ISO7816_IO_SEL_GPIOB                1
#define ISO7816_IO_SEL_AON                  2
#define ISO7816_IO_SEL_MSIO                 3
#define ISO7816_IO_CONFIG_SEL               ISO7816_IO_SEL_MSIO

#if (ISO7816_IO_CONFIG_SEL == ISO7816_IO_SEL_GPIOA)
    #define DEFAULT_IO_CONFIG               { { APP_IO_TYPE_GPIOA, APP_IO_MUX_6, APP_IO_PIN_9, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_GPIOA, APP_IO_MUX_6, APP_IO_PIN_7, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_GPIOA, APP_IO_MUX_6, APP_IO_PIN_8, APP_IO_PULLUP},\
                                              { APP_IO_TYPE_GPIOA, APP_IO_MUX_6, APP_IO_PIN_6, APP_IO_PULLUP} }
#elif (ISO7816_IO_CONFIG_SEL == ISO7816_IO_SEL_GPIOB)
    #define DEFAULT_IO_CONFIG               { { APP_IO_TYPE_GPIOB, APP_IO_MUX_2, APP_IO_PIN_5, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_GPIOB, APP_IO_MUX_2, APP_IO_PIN_7, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_GPIOB, APP_IO_MUX_2, APP_IO_PIN_6, APP_IO_PULLUP},\
                                              { APP_IO_TYPE_GPIOB, APP_IO_MUX_2, APP_IO_PIN_8, APP_IO_PULLUP} }
#elif (ISO7816_IO_CONFIG_SEL == ISO7816_IO_SEL_AON)
    #define DEFAULT_IO_CONFIG               { { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_3, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_1, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_2, APP_IO_PULLUP},\
                                              { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_0, APP_IO_PULLUP} }
#elif (ISO7816_IO_CONFIG_SEL == ISO7816_IO_SEL_MSIO)
    #define DEFAULT_IO_CONFIG               { { APP_IO_TYPE_MSIO, APP_IO_MUX_1, APP_IO_PIN_0, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_MSIO, APP_IO_MUX_1, APP_IO_PIN_2, APP_IO_NOPULL},\
                                              { APP_IO_TYPE_MSIO, APP_IO_MUX_1, APP_IO_PIN_1, APP_IO_PULLUP},\
                                              { APP_IO_TYPE_MSIO, APP_IO_MUX_1, APP_IO_PIN_3, APP_IO_PULLUP} }
#endif

#define ISO7816_CLK_MHZ(X)                  (SystemCoreClock / (1000000 *(X)) - 1)
#define CHANGE_BYTE_HIGH_LOW(X)             ((((X)<<4) & 0xF0) | (((X)>>4) & 0x0F))

app_iso7816_params_t params = {

    .use_mode = APP_ISO7816_TYPE_POLLING,
    .pin_cfg  = DEFAULT_IO_CONFIG,
    .init = {
        .wait_time     = 0x6B,
        .guard_time    = 0x0A,
        .detect_coding = ENABLE,
    },
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
iso7816_handle_t g_sim_card_handle = {0};

extern uint8_t ICCID_content[10];
extern uint8_t IMSI_content[9];

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_iso7816_evt_handler(app_iso7816_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_ISO7816_EVT_ERROR:
            printf("APP_ISO7816_EVT_ERROR \r\n");
            break;

        case APP_ISO7816_EVT_ABORT:
            printf("APP_ISO7816_EVT_ABORT \r\n");
            break;

        case APP_ISO7816_EVT_PRESENCE:
            printf("APP_ISO7816_EVT_PRESENCE \r\n");
            break;

        case APP_ISO7816_EVT_ATR_CPLT:
            printf("APP_ISO7816_EVT_ATR_CPLT \r\n");
            for (int i = 0; i < g_sim_card_handle.buffer_size; i++)
            {
                printf("%02x ", g_sim_card_handle.p_tx_rx_buffer[i]);
            }
            printf("\r\n");
            break;

        case APP_ISO7816_EVT_TX_CPLT:
            break;

        case APP_ISO7816_EVT_RX_CPLT:
            break;

        case APP_ISO7816_EVT_TX_RX_CPLT:
            break;
        default:
            break;
    }
}

void sim_card_demo(void)
{
    params.init.clk_div = ISO7816_CLK_MHZ(2);
    app_iso7816_init(&params, app_iso7816_evt_handler);

    g_sim_card_handle = *app_iso7816_get_handle();

    sim_card_get_ATR(&g_sim_card_handle);
    sim_card_PTS(&g_sim_card_handle);
    sim_card_select_MF(&g_sim_card_handle);

    sim_card_read_ICCID(&g_sim_card_handle);
    printf("\r\nICCID: ");
    for(uint8_t i=0;i<10;i++)
    {
      ICCID_content[i] = CHANGE_BYTE_HIGH_LOW(ICCID_content[i]);
      printf("%02x ",ICCID_content[i]);
    }

    sim_card_read_IMSI(&g_sim_card_handle);
    printf("\r\nIMSI: ");
    for(uint8_t i= 0;i<9;i++)
    {
      IMSI_content[i] = CHANGE_BYTE_HIGH_LOW(IMSI_content[i]);
      printf("%02x ",(IMSI_content[i]));
    }

    app_iso7816_deinit();
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("**********************************************************\r\n");
    printf("*                 SIM Card example.                      *\r\n");
    printf("*                                                        *\r\n");
    printf("*                                                        *\r\n");
    printf("*        (MSIO_PIN_0)  ----->    CLK                     *\r\n");
    printf("*        (MSIO_PIN_1) <----->    IO                      *\r\n");
    printf("*        (MSIO_PIN_2) <----->    RST                     *\r\n");
    printf("*        (MSIO_PIN_3) <----->    PRESENCE                *\r\n");
    printf("*                                                        *\r\n");
    printf("* Please connect SK boards and simcard.                  *\r\n");
    printf("* This smaple will show ISO7816 master read simcard data *\r\n");
    printf("**********************************************************\r\n");

    sim_card_demo();
    printf("\r\nThis example demo end.\r\n");

    while(1);
}
