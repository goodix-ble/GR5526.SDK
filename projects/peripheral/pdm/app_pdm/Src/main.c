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
#include "app_dma.h"
#include "app_pdm.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */

#define PDM_IO_MUX                  APP_PDM_LOOPBACK_IO_MUX
#define PDM_CLK_PIN                 APP_PDM_LOOPBACK_CLK_PIN
#define PDM_DATA_PIN                APP_PDM_LOOPBACK_DATA_PIN
#define PDM_CLK_TYPE                APP_PDM_LOOPBACK_CLK_TYPE
#define PDM_DATA_TYPE               APP_PDM_LOOPBACK_DATA_TYPE
#define DATA_LEN                    (4095)
#define BLOCKS_NUM                  (5)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t stereo_data[DATA_LEN * BLOCKS_NUM] = {0};
static volatile bool s_g_pdm_dma_cplt_flag = false;
app_pdm_params_t pdm_params = {
    .pin_cfg = {
        .clk = {
            .type = PDM_CLK_TYPE,
            .mux  = PDM_IO_MUX,
            .pin  = PDM_CLK_PIN,
            .pull = APP_IO_NOPULL,
        },
        .data = {
            .type = PDM_DATA_TYPE,
            .mux  = PDM_IO_MUX,
            .pin  = PDM_DATA_PIN,
            .pull = APP_IO_NOPULL,
        },
    },
    .dma_cfg = {
        .dma_instance = DMA1,
        .dma_channel  = DMA_Channel0,
    },
    .init = {
        .mode   = PDM_MODE_STEREO,
        .gain_l = 0x400U,
        .gain_r = 0x400U,
        .sample_rate = PDM_SAMPLE_RATE_16K,
    },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_pdm_evt_callback(app_pdm_evt_t *type)
{
    if(APP_PDM_EVT_DMA_TFR == type->type)
    {
        s_g_pdm_dma_cplt_flag = true;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_pdm_stereo_test(void)
{
    uint16_t ret = 0;

    ret = app_pdm_init(&pdm_params,app_pdm_evt_callback);
    if(APP_DRV_SUCCESS != ret)
    {
        return;
    }
    //begin sample data
    memset((uint32_t *)stereo_data,0,sizeof(stereo_data));
    dma_block_config_t block4 = {
        .src_address = (uint32_t)NULL,
        .dst_address = (uint32_t)&stereo_data[DATA_LEN * 4],
        .p_lli = NULL,
        .CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                         DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)(DATA_LEN),
        .src_status = 0x0,
        .dst_status = 0x0,
    };

    dma_block_config_t block3 = {
        .src_address = (uint32_t)NULL,
        .dst_address = (uint32_t)&stereo_data[DATA_LEN * 3],
        .p_lli = &block4,
        .CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                         DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)(DATA_LEN),
        .src_status = 0x0,
        .dst_status = 0x0,
    };
    dma_block_config_t block2 = {
        .src_address = (uint32_t)NULL,
        .dst_address = (uint32_t)&stereo_data[DATA_LEN * 2],
        .p_lli = &block3,
        .CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                         DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)(DATA_LEN),
        .src_status = 0x0,
        .dst_status = 0x0,
    };

    dma_block_config_t block1 = {
        .src_address = (uint32_t)NULL,
        .dst_address = (uint32_t)&stereo_data[DATA_LEN * 1],
        .p_lli = &block2,
        .CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                         DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)(DATA_LEN),
        .src_status = 0x0,
        .dst_status = 0x0,
    };
    dma_block_config_t block0 = {
        .src_address = (uint32_t)NULL,
        .dst_address = (uint32_t)&stereo_data[0],
        .p_lli = &block1,
        .CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                         DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)(DATA_LEN),
        .src_status = 0x0,
        .dst_status = 0x0,
    };
    dma_sg_llp_config_t sg_llp_config;
    sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_DISABLE;
    sg_llp_config.scatter_config.dst_dsi = (uint32_t)0;
    sg_llp_config.scatter_config.dst_dsc = (uint32_t)0;
    sg_llp_config.gather_config.src_gather_en = DMA_SRC_GATHER_DISABLE;
    sg_llp_config.gather_config.src_sgi = (uint32_t)0;
    sg_llp_config.gather_config.src_sgc = (uint32_t)0;
    sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_DISABLE;
    sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_ENABLE;
    sg_llp_config.llp_config.head_lli = &block0;

    s_g_pdm_dma_cplt_flag = false;
    app_pdm_stereo_start_dma_sg_llp(&stereo_data[0],DATA_LEN,&sg_llp_config);

    uint32_t delay_count = 0;
    while(s_g_pdm_dma_cplt_flag != true && delay_count++ < 500)
    {
        delay_ms(10);
    }
    app_pdm_deinit();
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   PDM example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("* This code will show how PDM sample data            *\r\n");
    printf("* stereo mode                                        *\r\n");
    printf("******************************************************\r\n");

    app_pdm_stereo_test();
    printf("\r\nThis example demo end.\r\n");

    while(1);
}
