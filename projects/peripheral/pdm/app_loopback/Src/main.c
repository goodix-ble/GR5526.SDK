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
#include "app_i2s.h"
#include "app_i2s_dma.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551x)
#define I2S_DMA                     DMA
#else
#define I2S_DMA                     DMA1
#endif

#define I2S_IO_MUX                  APP_I2S_LOOPBACK_IO_MUX
#define I2S_WS_PIN                  APP_I2S_LOOPBACK_WS_PIN
#define I2S_SDO_PIN                 APP_I2S_LOOPBACK_SDO_PIN
#define I2S_SDI_PIN                 APP_I2S_LOOPBACK_SDI_PIN
#define I2S_SCLK_PIN                APP_I2S_LOOPBACK_SCLK_PIN
#define I2S_WS_TYPE                 APP_I2S_LOOPBACK_WS_TYPE
#define I2S_SDO_TYPE                APP_I2S_LOOPBACK_SDO_TYPE
#define I2S_SDI_TYPE                APP_I2S_LOOPBACK_SDI_TYPE
#define I2S_SCLK_TYPE               APP_I2S_LOOPBACK_SCLK_TYPE
#define PDM_IO_MUX                  APP_PDM_LOOPBACK_IO_MUX
#define PDM_CLK_PIN                 APP_PDM_LOOPBACK_CLK_PIN
#define PDM_DATA_PIN                APP_PDM_LOOPBACK_DATA_PIN
#define PDM_CLK_TYPE                APP_PDM_LOOPBACK_CLK_TYPE
#define PDM_DATA_TYPE               APP_PDM_LOOPBACK_DATA_TYPE

#define PDM_DMA_BLOCK_NUM           (2)
#define PDM_AUDIO_FRAME_LEN         (240)
#define PDM_CHANNEL_NUM             (2)
#define PDM_TX_BLOCK_STATE          (0x11)

#define I2S_DMA_BLOCK_NUM           (2)
#define I2S_AUDIO_FRAME_LEN         (PDM_AUDIO_FRAME_LEN)
#define I2S_CHANNEL_NUM             (PDM_CHANNEL_NUM)

typedef struct
{
    dma_sg_llp_config_t sg_llp_config;
    dma_block_config_t dma_block[PDM_DMA_BLOCK_NUM];
}LOOPBACK_DMA_LLP_T;

typedef struct
{
    volatile uint32_t pdm_block_index;
    volatile uint32_t pdm_audio_rx_state;
    volatile uint32_t i2s_tdone;
    uint32_t pdm_audio_buf[PDM_AUDIO_FRAME_LEN * PDM_DMA_BLOCK_NUM];
    uint16_t pdm_audio_left_buf[PDM_AUDIO_FRAME_LEN];
    uint16_t pdm_audio_right_buf[PDM_AUDIO_FRAME_LEN];
    LOOPBACK_DMA_LLP_T pdm_dma;
    uint8_t i2s_streaming_state;
    uint16_t i2s_audio_buf[I2S_AUDIO_FRAME_LEN * I2S_CHANNEL_NUM * I2S_DMA_BLOCK_NUM];
}LOOPBACK_EXAMPLE_T;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
LOOPBACK_EXAMPLE_T g_Loopback_ctrl;

app_pdm_params_t g_pdm_params = {
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
        .gain_l = 0x800U, // gain(db): 20 * log(0x400/1024) = 0db, 20 * log(0x800/1024) = 6db
        .gain_r = 0x800U, // gain(db): 20 * log(0x400/1024) = 0db, 20 * log(0x800/1024) = 6db
        .sample_rate = PDM_SAMPLE_RATE_16K,
    },
};

app_i2s_params_t g_i2s_master_params = {
   .id = APP_I2S_ID_MASTER,
   .pin_cfg = {
       .ws = {
           .type = I2S_WS_TYPE,
           .mux  = I2S_IO_MUX,
           .pin  = I2S_WS_PIN,
           .pull = APP_IO_NOPULL,
       },
       .sdo = {
           .type = I2S_SDO_TYPE,
           .mux  = I2S_IO_MUX,
           .pin  = I2S_SDO_PIN,
           .pull = APP_IO_NOPULL,
       },
       .sdi = {
           .type = I2S_SDI_TYPE,
           .mux  = I2S_IO_MUX,
           .pin  = I2S_SDI_PIN,
           .pull = APP_IO_NOPULL,
       },
       .sclk = {
           .type = I2S_SCLK_TYPE,
           .mux  = I2S_IO_MUX,
           .pin  = I2S_SCLK_PIN,
           .pull = APP_IO_NOPULL,
       },
   },
   .dma_cfg = {
       .tx_dma_instance = I2S_DMA,
       .rx_dma_instance = I2S_DMA,
       .tx_dma_channel  = DMA_Channel2,
       .rx_dma_channel  = DMA_Channel3,
   },
   .init = {
       .data_size    = I2S_DATASIZE_16BIT,
       .clock_source = I2S_CLOCK_SRC_64M,
       .ws_cycles    = I2S_WS_CYCLES_16,
       .audio_freq   = 16000,
   },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static LOOPBACK_EXAMPLE_T *loopback_ctrl_get(void)
{
    return &g_Loopback_ctrl;
}

static void loopback_pdm_dma_evt_callback(app_pdm_evt_t *type)
{
    switch(type->type)
    {
        case APP_PDM_EVT_DMA_TFR:
        {
            g_Loopback_ctrl.pdm_audio_rx_state = true;
        }
        break;

        case APP_PDM_EVT_DMA_BLK:
        {
            if(g_Loopback_ctrl.pdm_dma.dma_block[0].dst_status == PDM_TX_BLOCK_STATE)
            {
                g_Loopback_ctrl.pdm_dma.dma_block[0].dst_status = 0;
                g_Loopback_ctrl.pdm_block_index = 0;
            }

            if(g_Loopback_ctrl.pdm_dma.dma_block[1].dst_status == PDM_TX_BLOCK_STATE)
            {
                g_Loopback_ctrl.pdm_dma.dma_block[1].dst_status = 0;
                g_Loopback_ctrl.pdm_block_index = 1;
            }

            g_Loopback_ctrl.pdm_audio_rx_state = true;
        }
        break;

        case APP_PDM_EVT_LEFT_OVERFLOW:
        {
        }
        break;

        case APP_PDM_EVT_RIGHT_OVERFLOW:
        {
        }
        break;

        case APP_PDM_EVT_DMA_ERROR:
        {
        }
        break;

        default:
        break;
    }
}

static void loopback_i2s_dma_evt_callback(app_i2s_evt_t *p_evt)
{
    switch(p_evt->type)
    {
        case APP_I2S_EVT_TX_CPLT:
        {
            g_Loopback_ctrl.i2s_tdone = 1;
        }
        break;

        case APP_I2S_EVT_RX_DATA:
        {
        }
        break;

        case APP_I2S_EVT_TX_RX:
        {
        }
        break;

        case APP_I2S_EVT_ERROR:
        {
        }
        break;

        default:
        break;
    }
}

static uint16_t loopback_pdm_dma_init(LOOPBACK_DMA_LLP_T *pdm_dma, void *pdm_audio_buf, uint32_t pdm_audio_len)
{
    if(pdm_dma == NULL || pdm_audio_buf == NULL)
    {
        return 1;
    }

    uint32_t *buf_p = (uint32_t *)pdm_audio_buf;

    pdm_dma->dma_block[0].src_address = (uint32_t)NULL;
    pdm_dma->dma_block[0].dst_address = (uint32_t)&buf_p[0];
    pdm_dma->dma_block[0].p_lli = &pdm_dma->dma_block[1];
    pdm_dma->dma_block[0].CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                                     DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                                     DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY;
    pdm_dma->dma_block[0].CTL_H = (uint32_t)(pdm_audio_len);
    pdm_dma->dma_block[0].src_status = 0x0;
    pdm_dma->dma_block[0].dst_status = 0x0;

    pdm_dma->dma_block[1].src_address = (uint32_t)NULL;
    pdm_dma->dma_block[1].dst_address = (uint32_t)&buf_p[pdm_audio_len];
    pdm_dma->dma_block[1].p_lli = &pdm_dma->dma_block[0];
    pdm_dma->dma_block[1].CTL_L = DMA_CTLL_INI_EN|DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD |\
                                     DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE |\
                                     DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY;
    pdm_dma->dma_block[1].CTL_H = (uint32_t)(pdm_audio_len);
    pdm_dma->dma_block[1].src_status = 0x0;
    pdm_dma->dma_block[1].dst_status = 0x0;

    pdm_dma->sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_DISABLE;
    pdm_dma->sg_llp_config.scatter_config.dst_dsi = (uint32_t)0;
    pdm_dma->sg_llp_config.scatter_config.dst_dsc = (uint32_t)0;
    pdm_dma->sg_llp_config.gather_config.src_gather_en = DMA_SRC_GATHER_DISABLE;
    pdm_dma->sg_llp_config.gather_config.src_sgi = (uint32_t)0;
    pdm_dma->sg_llp_config.gather_config.src_sgc = (uint32_t)0;
    pdm_dma->sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_DISABLE;
    pdm_dma->sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_ENABLE;
    pdm_dma->sg_llp_config.llp_config.llp_src_writeback = 1;
    pdm_dma->sg_llp_config.llp_config.llp_dst_writeback = PDM_TX_BLOCK_STATE;
    pdm_dma->sg_llp_config.llp_config.head_lli = &pdm_dma->dma_block[0];

    return APP_DRV_SUCCESS;
}

static uint16_t loopback_pdm_init(void)
{
    uint16_t ret = 0;

    ret = app_pdm_init(&g_pdm_params, loopback_pdm_dma_evt_callback);
    if(APP_DRV_SUCCESS != ret)
    {
        printf("loopback_pdm_init fail, ret = %d\r\n", ret);
        return ret;
    }

    LOOPBACK_EXAMPLE_T *loopback_ctrl_p = loopback_ctrl_get();

    ret = loopback_pdm_dma_init(&loopback_ctrl_p->pdm_dma, loopback_ctrl_p->pdm_audio_buf, PDM_AUDIO_FRAME_LEN);
    if(APP_DRV_SUCCESS != ret)
    {
        printf("loopback_pdm_dma_init fail, ret = %d\r\n", ret);
        return ret;
    }

    return ret;
}

static uint16_t loopback_i2s_master_init(void)
{
    uint16_t ret = 0;

    ret = app_i2s_init(&g_i2s_master_params, loopback_i2s_dma_evt_callback);
    if (ret != 0)
    {
        printf("\r\nI2S M initial failed %d! Please check the input paraments.\r\n", ret);
    }

    ret = app_i2s_dma_init(&g_i2s_master_params);
    if (ret != 0)
    {
        printf("\r\nI2S M dma initial failed %d! Please check the input paraments.\r\n", ret);
    }

    return ret;
}

static uint16_t loopback_i2s_stereo_start(uint32_t block_num, uint16_t *p_data, uint16_t size)
{
    uint16_t ret = 0;

    if(g_Loopback_ctrl.i2s_streaming_state == 0)
    {
        g_Loopback_ctrl.i2s_streaming_state = 1;

        app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);

        g_Loopback_ctrl.i2s_tdone = 0;
        ret = app_i2s_dma_transmit_async(APP_I2S_ID_MASTER, p_data, size);
    }
    else
    {
        while(g_Loopback_ctrl.i2s_tdone == 0);

        g_Loopback_ctrl.i2s_tdone = 0;
        ret = app_i2s_dma_transmit_async(APP_I2S_ID_MASTER, p_data, size);
    }

    return ret;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void loopback_example_start(void)
{
    uint16_t ret = 0;
    LOOPBACK_EXAMPLE_T *loopback_ctrl_p = loopback_ctrl_get();

    loopback_ctrl_p->i2s_tdone = 0;
    loopback_ctrl_p->pdm_block_index = 0;
    loopback_ctrl_p->i2s_streaming_state = 0;
    loopback_ctrl_p->pdm_audio_rx_state = false;

    ret = app_pdm_stereo_start_dma_sg_llp(loopback_ctrl_p->pdm_audio_buf, PDM_AUDIO_FRAME_LEN, &loopback_ctrl_p->pdm_dma.sg_llp_config);

    memset(loopback_ctrl_p->i2s_audio_buf, 0, sizeof(loopback_ctrl_p->i2s_audio_buf));

    if(ret != APP_DRV_SUCCESS)
    {
        printf("loopback_i2s_dma_start fail, ret = %d\r\n", ret);
    }

    printf("loopback_pdm_dma_start! \r\n");

    uint32_t cnt = 0, block_num = 0;

    while(1)
    {
        if(loopback_ctrl_p->pdm_audio_rx_state == true)
        {
            cnt = 0;
            loopback_ctrl_p->pdm_audio_rx_state = false;
            block_num = loopback_ctrl_p->pdm_block_index;

            while(cnt < PDM_AUDIO_FRAME_LEN)
            {
                loopback_ctrl_p->pdm_audio_left_buf[cnt] = (loopback_ctrl_p->pdm_audio_buf[block_num * PDM_AUDIO_FRAME_LEN + cnt] >> 16) & 0xffff;
                loopback_ctrl_p->pdm_audio_right_buf[cnt] = loopback_ctrl_p->pdm_audio_buf[block_num * PDM_AUDIO_FRAME_LEN + cnt] & 0xffff;

                loopback_ctrl_p->i2s_audio_buf[block_num * I2S_AUDIO_FRAME_LEN * I2S_CHANNEL_NUM + I2S_CHANNEL_NUM * cnt] = loopback_ctrl_p->pdm_audio_right_buf[cnt];
                loopback_ctrl_p->i2s_audio_buf[block_num * I2S_AUDIO_FRAME_LEN * I2S_CHANNEL_NUM + I2S_CHANNEL_NUM * cnt + 1] = loopback_ctrl_p->pdm_audio_left_buf[cnt];

                cnt++;
            }

            ret = loopback_i2s_stereo_start(block_num, &loopback_ctrl_p->i2s_audio_buf[block_num * I2S_AUDIO_FRAME_LEN * I2S_CHANNEL_NUM], I2S_AUDIO_FRAME_LEN);

            if(ret != APP_DRV_SUCCESS)
            {
                printf("loopback_i2s_stereo fail, ret = %d\r\n", ret);
            }
        }
    }
}

void loopback_example_init(void)
{
    uint16_t ret = loopback_pdm_init();
    printf("loopback_pdm_init ret = %d\r\n", ret);

    ret = loopback_i2s_master_init();
    printf("loopback_i2s_master_init ret = %d\r\n", ret);

    return;
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   Loopback example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* This code will show how PDM sample data to         *\r\n");
    printf("* codec chip through i2s in stereo mode              *\r\n");
    printf("******************************************************\r\n");

    loopback_example_init();
    loopback_example_start();
    printf("\r\nThis example demo end.\r\n");

    while(1);
}
