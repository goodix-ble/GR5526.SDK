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
#include "app_dspi.h"
#include "app_dspi_dma.h"
#include "board_SK.h"
#include "grx_hal.h"
#include "Bird.h"
#include "HeartRate.h"
#include "Music.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DSPI_DATA_LEN                       (16)
#define DSPI_POXEL_LEN                      (57600)
#define DSPI_DMA_SIZE                       (4094 * 2)
#define DSPI_SOFT_CS_MODE_ENABLE            1u

#define DSPI_DATA_SRAM
#define DSPI_ST7789V_X_START                0
#define DSPI_ST7789V_X_END                  239
#define DSPI_ST7789V_Y_START                40
#define DSPI_ST7789V_Y_END                  319

/*
 * LOCAL VARIABLE DEFINITIONS
 ***************************************************************
 *
 */
static volatile uint8_t                     g_master_tdone = 0;
static dspi_command_t                       g_cmd;
static uint8_t                              s_data_8[DSPI_DATA_LEN] = {0};
static uint16_t                             s_data_16[DSPI_DATA_LEN] = {0};
#if defined(DSPI_DATA_SRAM)
static uint16_t                             pixel[DSPI_POXEL_LEN] = {0};
#endif
static volatile uint8_t                     display_busy = 0;
static volatile uint32_t                    display_xfer_remain = 0;
static uint8_t                              *display_xfer_point = NULL;

app_dspi_params_t dspi_params = {
    .pin_cfg = {
        .cs = {
            .type   = APP_DSPI_CS_TYPE,
            .mux    = APP_DSPI_CS_PIN_MUX,
            .pin    = APP_DSPI_CS_PIN,
            .pull   = APP_DSPI_CS_PULL,
            .enable = APP_DSPI_CS_ENABLE,
        },
        .clk = {
            .type   = APP_DSPI_CLK_TYPE,
            .mux    = APP_DSPI_CLK_PIN_MUX,
            .pin    = APP_DSPI_CLK_PIN,
            .pull   = APP_DSPI_CLK_PULL,
            .enable = APP_DSPI_CLK_ENABLE,
        },
        .mosi = {
            .type   = APP_DSPI_MOSI_TYPE,
            .mux    = APP_DSPI_MOSI_PIN_MUX,
            .pin    = APP_DSPI_MOSI_PIN,
            .pull   = APP_DSPI_MOSI_PULL,
            .enable = APP_DSPI_MOSI_ENABLE,
        },
        .miso = {
            .type   = APP_DSPI_MISO_TYPE,
            .mux    = APP_DSPI_MISO_PIN_MUX,
            .pin    = APP_DSPI_MISO_PIN,
            .pull   = APP_DSPI_MISO_PULL,
            .enable = APP_DSPI_MISO_ENABLE,
        },
        .dcx = {
            .type   = APP_DSPI_DCX_TYPE,
            .mux    = APP_DSPI_DCX_PIN_MUX,
            .pin    = APP_DSPI_DCX_PIN,
            .pull   = APP_DSPI_DCX_PULL,
            .enable = APP_DSPI_DCX_ENABLE,
        },
    },
    .dma_cfg.channel = DMA_Channel0,
    .init = {
        .data_size = DSPI_DATASIZE_08_BITS,
        .baud_rate = DSPI_BAUD_RATE_8P1PCLK,
        .dspi_mode = DSPI_PROT_MODE_3W1L,
    },
    .is_soft_cs = true,
};

/*
 * LOCAL FUNCTION
 ***************************************************************
 *
 */
static void app_dspi_display_xfer_start(uint8_t *data, uint32_t dataLen)
{
    app_dspi_dma_transmit_async(data, dataLen);
}

static void app_dspi_evt_handler(app_dspi_evt_t *p_evt)
{
    if (p_evt->type == APP_DSPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
        if (display_busy == 1)
        {
            if (display_xfer_remain != 0)
            {
                if (display_xfer_remain > DSPI_DMA_SIZE)
                {
                    app_dspi_display_xfer_start(display_xfer_point, DSPI_DMA_SIZE);
                    display_xfer_remain -= DSPI_DMA_SIZE;
                    display_xfer_point += DSPI_DMA_SIZE;
                }
                else
                {
                    app_dspi_display_xfer_start(display_xfer_point, DSPI_DMA_SIZE);
                    display_xfer_remain = 0;
                    display_xfer_point += display_xfer_remain;
                }
            }
            else
            {
                display_busy = 0;
            }
        }
    }

    if (p_evt->type == APP_DSPI_EVT_ERROR)
    {
    }
}

static void app_dspi_display_coordinates(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    // Write_Command(0x2A);
    g_cmd.instruction = 0x2A00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*4;

    g_master_tdone = 0;
    s_data_16[0] = ((x1 & 0xFF00));
    s_data_16[1] = ((x1 & 0x00FF) << 8);
    s_data_16[2] = ((x2 & 0xFF00));
    s_data_16[3] = ((x2 & 0x00FF) << 8);
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0x2B);
    g_cmd.instruction = 0x2B00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*4;

    g_master_tdone = 0;
    s_data_16[0] = ((y1 & 0xFF00));
    s_data_16[1] = ((y1 & 0x00FF) << 8);
    s_data_16[2] = ((y2 & 0xFF00));
    s_data_16[3] = ((y2 & 0x00FF) << 8);
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);
}

static void app_dspi_display_write(uint8_t *data, uint32_t dataLen)
{
    // Write_Command(0x2C);
    g_cmd.instruction = 0x2C00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;
    g_master_tdone = 0;

    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);

    display_xfer_remain = dataLen;
    display_xfer_point = data;
    display_busy = 1;

    app_dspi_config_data_size(DSPI_DATASIZE_16_BITS);

    if (display_xfer_remain > DSPI_DMA_SIZE)
    {
        app_dspi_display_xfer_start(display_xfer_point, DSPI_DMA_SIZE);
        display_xfer_remain -= DSPI_DMA_SIZE;
        display_xfer_point += DSPI_DMA_SIZE;
    }
    else
    {
        app_dspi_display_xfer_start(data, dataLen);
        display_xfer_remain = 0;
        display_xfer_point += dataLen;
    }
}

static void app_dspi_display_init(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    uint16_t ret = APP_DRV_ERR_HAL;

    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pull = GPIO_PULLDOWN;
    gpio_config.pin  = GPIO_PIN_8 | GPIO_PIN_9;
    hal_gpio_init(GPIO0, &gpio_config);

    hal_gpio_write_pin(GPIO0, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    ret = app_dspi_init(&dspi_params, app_dspi_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Initialize DSPI failed\r\n");
    }
    delay_ms(100);

    ret = app_dspi_dma_init(&dspi_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Initialize DSPI DMA failed\r\n");
    }
    delay_ms(100);

    // Write_Command(0x11);
    g_cmd.instruction = 0x11;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_08_BITS;
    g_cmd.length = 1;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);

    while(g_master_tdone == 0);
    delay_ms(100);

    // Write_Command(0xE7);
    g_cmd.instruction = 0xE7;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_08_BITS;
    g_cmd.length = 1;

    g_master_tdone = 0;
    s_data_8[0] = 0x10;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_8);
    while(g_master_tdone == 0);

    // 4W2L
    app_dspi_config_mode(DSPI_PROT_MODE_4W2L);
    app_dspi_config_data_size(DSPI_DATASIZE_16_BITS);

    // Write_Command(0x36);
    g_cmd.instruction = 0x3600;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x0000;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0x3A);
    g_cmd.instruction = 0x3A00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x0500;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xB2);
    g_cmd.instruction = 0xB200;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2 * 5;

    g_master_tdone = 0;
    s_data_16[0] = 0x0C00;
    s_data_16[1] = 0x0C00;
    s_data_16[2] = 0x0000;
    s_data_16[3] = 0x3300;
    s_data_16[4] = 0x3300;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xB7);
    g_cmd.instruction = 0xB700;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x3500;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xBB);
    g_cmd.instruction = 0xBB00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x1E00;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xC0);
    g_cmd.instruction = 0xC000;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x2C00;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xC2);
    g_cmd.instruction = 0xC200;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x0100;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xC3);
    g_cmd.instruction = 0xC300;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x0300;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xC4);
    g_cmd.instruction = 0xC400;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x2000;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xC6);
    g_cmd.instruction = 0xC600;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    s_data_16[0] = 0x0F00;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xD0);
    g_cmd.instruction = 0xD000;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*2;

    g_master_tdone = 0;
    s_data_16[0] = 0xA400;
    s_data_16[1] = 0xA100;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xE0);
    g_cmd.instruction = 0xE000;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*14;

    g_master_tdone = 0;
    s_data_16[0] = 0xD000;
    s_data_16[1] = 0x0600;
    s_data_16[2] = 0x0C00;
    s_data_16[3] = 0x0800;
    s_data_16[4] = 0x0800;
    s_data_16[5] = 0x2500;
    s_data_16[6] = 0x3300;
    s_data_16[7] = 0x3F00;
    s_data_16[8] = 0x4F00;
    s_data_16[9] = 0x2900;
    s_data_16[10] = 0x1700;
    s_data_16[11] = 0x1400;
    s_data_16[12] = 0x3300;
    s_data_16[13] = 0x3200;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0xE1);
    g_cmd.instruction = 0xE100;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*14;

    g_master_tdone = 0;
    s_data_16[0] = 0xD000;
    s_data_16[1] = 0x0700;
    s_data_16[2] = 0x0C00;
    s_data_16[3] = 0x0900;
    s_data_16[4] = 0x0800;
    s_data_16[5] = 0x2500;
    s_data_16[6] = 0x3400;
    s_data_16[7] = 0x3800;
    s_data_16[8] = 0x4E00;
    s_data_16[9] = 0x2900;
    s_data_16[10] = 0x1700;
    s_data_16[11] = 0x1400;
    s_data_16[12] = 0x3200;
    s_data_16[13] = 0x3200;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0x2A);
    g_cmd.instruction = 0x2A00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*4;

    g_master_tdone = 0;
    s_data_16[0] = 0x0000;
    s_data_16[1] = 0x0000;
    s_data_16[2] = 0x0000;
    s_data_16[3] = 0xEF00;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0x2B);
    g_cmd.instruction = 0x2B00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2*4;

    g_master_tdone = 0;
    s_data_16[0] = 0x0000;
    s_data_16[1] = 0x0000;
    s_data_16[2] = 0x0000;
    s_data_16[3] = 0xEF00;
    app_dspi_dma_command_transmit_async(&g_cmd, (uint8_t *)s_data_16);
    while(g_master_tdone == 0);

    // Write_Command(0x21);
    g_cmd.instruction = 0x2100;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);

    // Write_Command(0x29);
    g_cmd.instruction = 0x2900;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);

    // Write_Command(0x2C);
    g_cmd.instruction = 0x2C00;
    g_cmd.instruction_size = DSPI_INSTSIZE_16_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 2;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);
}

static void app_dspi_display(void)
{
    while(1)
    {
    #if defined(DSPI_DATA_SRAM)
        for (uint16_t j = 0; j < DSPI_POXEL_LEN; j++)
        {
            pixel[j] = (uint16_t)bird[j];
        }
    #endif

        printf("APP DISPLAY BIRD\r\n");
        app_dspi_display_coordinates(DSPI_ST7789V_X_START, DSPI_ST7789V_X_END, DSPI_ST7789V_Y_START, DSPI_ST7789V_Y_END);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_RESET);
    #if !defined(DSPI_DATA_SRAM)
        app_dspi_display_write((uint8_t *)bird, DSPI_POXEL_LEN * 2);
    #else
        app_dspi_display_write((uint8_t *)pixel, DSPI_POXEL_LEN * 2);
    #endif
        while(display_busy == 1);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_SET);
        delay_ms(1000);

    #if defined(DSPI_DATA_SRAM)
        for (uint16_t j = 0; j < DSPI_POXEL_LEN; j++)
        {
            pixel[j] = (uint16_t)HeartRate[j];
        }
    #endif

        printf("APP DISPLAY HEARTRATE\r\n");
        app_dspi_display_coordinates(DSPI_ST7789V_X_START, DSPI_ST7789V_X_END, DSPI_ST7789V_Y_START, DSPI_ST7789V_Y_END);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_RESET);
    #if !defined(DSPI_DATA_SRAM)
        app_dspi_display_write((uint8_t *)HeartRate, DSPI_POXEL_LEN * 2);
    #else
        app_dspi_display_write((uint8_t *)pixel, DSPI_POXEL_LEN * 2);
    #endif
        while(display_busy == 1);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_SET);
        delay_ms(1000);

    #if defined(DSPI_DATA_SRAM)
        for (uint16_t j = 0; j < DSPI_POXEL_LEN; j++)
        {
            pixel[j] = (uint16_t)Music[j];
        }
    #endif

        printf("APP DISPLAY MUSIC\r\n");
        app_dspi_display_coordinates(DSPI_ST7789V_X_START, DSPI_ST7789V_X_END, DSPI_ST7789V_Y_START, DSPI_ST7789V_Y_END);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_RESET);
    #if !defined(DSPI_DATA_SRAM)
        app_dspi_display_write((uint8_t *)Music, DSPI_POXEL_LEN * 2);
    #else
        app_dspi_display_write((uint8_t *)pixel, DSPI_POXEL_LEN * 2);
    #endif
        while(display_busy == 1);
        hal_gpio_write_pin(GPIO0, GPIO_PIN_9, GPIO_PIN_SET);
        delay_ms(1000);
    }
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              DSPI_DISPLAY_APP example.             *\r\n");
    printf("*                                                    *\r\n");
    printf("*             SCLK: 2P1PCLK MODE: 4W2L               *\r\n");
    printf("*   LCD: ST7789V RESOLUTION:240*240 PIXEL:RGB(565)   *\r\n");
    printf("*                                                    *\r\n");
    printf("*           DSPI         ----->    ST7789V           *\r\n");
    printf("*   DSPI_CS  (CS_PIN)    ----->    CS                *\r\n");
    printf("*   DSPI_CLK (CLK_PIN)   ----->    CLK               *\r\n");
    printf("*   DSPI_MOSI(MOSI_PIN)  ----->    SDA               *\r\n");
    printf("*   DSPI_DCX (DCX_PIN)   ----->    DCX               *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will use the DSPI interface       *\r\n");
    printf("* to refresh the screen. Please use ST7789V LCD      *\r\n");
    printf("******************************************************\r\n");

    app_dspi_display_init();

    app_dspi_display();

    while(1);
}
