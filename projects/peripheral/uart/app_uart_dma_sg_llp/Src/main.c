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
#include "app_uart.h"
#include "app_uart_dma.h"
#include "board_SK.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_DATA_LEN                       (128)
#define UART_ID                             APP_UART1_ID

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_ring_buffer[UART_DATA_LEN * 2] = {0};

volatile uint8_t g_tdone = 0;
volatile uint8_t g_rdone = 0;
volatile uint16_t rlen = 0;
uint8_t g_tx_buffer[UART_DATA_LEN] = {0};
uint8_t g_rx_buffer[UART_DATA_LEN] = {0};

app_uart_params_t uart_params = {
    .id      = UART_ID,
    .pin_cfg = {
           .tx = {
              .type = APP_UART1_TX_IO_TYPE,
              .mux  = APP_UART1_TX_PINMUX,
              .pin  = APP_UART1_TX_PIN,
              .pull = APP_IO_PULLUP,
           },
           .rx = {
              .type = APP_UART1_RX_IO_TYPE,
              .mux  = APP_UART1_RX_PINMUX,
              .pin  = APP_UART1_RX_PIN,
              .pull = APP_IO_PULLUP,
           },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DMA_Channel0,
        .rx_dma_channel  = DMA_Channel1 ,
    },
    .init = {
        .baud_rate = 115200,
        .data_bits = UART_DATABITS_8,
        .stop_bits = UART_STOPBITS_1,
        .parity    = UART_PARITY_NONE,
        .hw_flow_ctrl    = UART_HWCONTROL_NONE,
        .rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE,
    },
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_uart_callback(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_TX_CPLT)
    {
        g_tdone = 1;
    }
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        g_rdone = 1;
        rlen = p_evt->data.size;
        memcpy(g_tx_buffer, g_rx_buffer, rlen);
    }
    if (p_evt->type == APP_UART_EVT_ERROR)
    {
        g_tdone = 1;
        g_rdone = 1;
    }
    return;
}

/* The test case is from memory to TX, after RX receiving to check whether the data sent by TX meets expectations. */
/* SG config:  Gather Enable Gather count 1, Gather Interval 1*/
/*             Scatter Disable Scatter count 0, Scatter count 0*/
/* LLP config: Source LLP Enable, Source status write back 0xa*/
/*             Destination LLP Disable,Destination status write back 0x0*/
/*             Two blocks, llp head &block0*/
void app_uart_transmit_dma_sg_llp_test(void)
{
    uint16_t ret = 0;
    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = g_ring_buffer;
    uart_buffer.tx_buf_size = sizeof(g_ring_buffer);

    ret = app_uart_init(&uart_params, app_uart_callback, &uart_buffer);
    if (ret != APP_DRV_SUCCESS) 
    {
        printf("app_uart_init failed ret = %d\n",ret);
        return ;
    }

    ret = app_uart_dma_init(&uart_params);
    if (ret != APP_DRV_SUCCESS) 
    {
        printf("app_uart_dam_init failed ret = %d\n",ret);
        return ;
    }    
    
    uint8_t i = 0;
    uint8_t src_data0[UART_DATA_LEN] = {0};
    uint8_t src_data1[UART_DATA_LEN] = {0};
    uint8_t rx_data[UART_DATA_LEN] = {0};
    uint8_t check_data[UART_DATA_LEN] = {0};
    for(i = 0;i < sizeof(src_data0);i++)
    {
        src_data0[i] = i;
        src_data1[i] = i+UART_DATA_LEN;
        check_data[i] = i*2;
    }
    dma_block_config_t block1 = {
        .src_address = (uint32_t)&src_data1,
        .dst_address = (uint32_t)0,
        .p_lli = NULL,
        .CTL_L = DMA_CTLL_INI_EN | DMA_DST_NO_CHANGE | DMA_SRC_INCREMENT | DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE |\
                         DMA_SRC_GATHER_ENABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_ENABLE | DMA_LLP_DST_DISABLE | DMA_MEMORY_TO_PERIPH,
        .CTL_H = (uint32_t)UART_DATA_LEN/2,
        .src_status = 0x0,
        .dst_status = 0x0,
    };

    dma_block_config_t block0 = {
        .src_address = (uint32_t)&src_data0,
        .dst_address = (uint32_t)0,
        .p_lli = &block1,
        .CTL_L = DMA_CTLL_INI_EN | DMA_DST_NO_CHANGE | DMA_SRC_INCREMENT | DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE |\
                         DMA_SRC_GATHER_ENABLE | DMA_DST_SCATTER_DISABLE |\
                         DMA_LLP_SRC_ENABLE | DMA_LLP_DST_DISABLE | DMA_MEMORY_TO_PERIPH,
        .CTL_H = (uint32_t)UART_DATA_LEN/2,
        .src_status = 0x0,
        .dst_status = 0x0,
    };
    dma_sg_llp_config_t sg_llp_config;
    sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_DISABLE;
    sg_llp_config.scatter_config.dst_dsi = (uint32_t)0;
    sg_llp_config.scatter_config.dst_dsc = (uint32_t)0;
    sg_llp_config.gather_config.src_gather_en = DMA_SRC_GATHER_ENABLE;
    sg_llp_config.gather_config.src_sgi = (uint32_t)1;
    sg_llp_config.gather_config.src_sgc = (uint32_t)1;
    sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_ENABLE;
    sg_llp_config.llp_config.llp_src_writeback = 0xa;
    sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_DISABLE;
    sg_llp_config.llp_config.head_lli = &block0;
    memset(rx_data,0,sizeof(rx_data));
    g_tdone = 0;
    g_rdone = 0;

    app_uart_dma_receive_async(UART_ID, rx_data, UART_DATA_LEN);
    app_uart_transmit_dma_sg_llp(UART_ID, src_data0, UART_DATA_LEN/2,&sg_llp_config);
    uint8_t delay_count = 100;
    while((g_rdone != 1) && (g_tdone != 1) && (delay_count-- > 0)) 
    {
        delay_ms(50);
    }
    if(g_rdone != 1 || g_tdone != 1)
    {
        printf("uart_transmit_dma_sg_llp transmit failed\r\n");
    } 
    else
    {
        printf("uart_transmit_dma_sg_llp transmit success\r\n");
    }
    if(0 != strncmp((const char*)&check_data,(const char*)&rx_data,UART_DATA_LEN)) 
    {
        printf("uart_transmit_dma_sg_llp data error\r\n");
    }
    else
    {
        printf("uart_transmit_dma_sg_llp data correct\r\n");
    }
    if(sg_llp_config.llp_config.llp_src_writeback != block0.src_status ||
       sg_llp_config.llp_config.llp_src_writeback != block1.src_status ) 
    {
        printf("uart_transmit_dma_sg_llp write back error\r\n");
    } 
    else
    {
        printf("uart_transmit_dma_sg_llp write back success\r\n");
    }
    app_log_flush();
    app_uart_deinit(UART_ID);
    app_uart_dma_deinit(UART_ID);
}

/* The test case is from RX to memory, checking whether the data sent by TX meets expectations. */
/* SG config:  Gather Disable Gather count 0, Gather Interval 0*/
/*             Scatter Enable Scatter count 1, Scatter count 1*/
/* LLP config: Source LLP Disable, Source status write back 0x0*/
/*             Destination LLP Enable,Destination status write back 0xB*/
/*             Two blocks, llp head &block0*/
void app_uart_receive_dma_sg_llp_test(void)
{
    uint16_t ret = 0;
    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = g_ring_buffer;
    uart_buffer.tx_buf_size = sizeof(g_ring_buffer);

    ret = app_uart_init(&uart_params, app_uart_callback, &uart_buffer);
    if (ret != APP_DRV_SUCCESS) 
    {
        printf("app_uart_init failed ret = %d\r\n",ret);
        return ;
    }

    ret = app_uart_dma_init(&uart_params);
    if (ret != APP_DRV_SUCCESS) 
    {
        printf("app_uart_dam_init failed ret = %d\n",ret);
        return ;
    }

    uint8_t i = 0;
    uint8_t tx_data[UART_DATA_LEN] = {0};
    uint8_t dst_data0[UART_DATA_LEN] = {0};
    uint8_t dst_data1[UART_DATA_LEN] = {0};
    uint8_t check_data0[UART_DATA_LEN] = {0};
    uint8_t check_data1[UART_DATA_LEN] = {0};
    for(i = 0;i < sizeof(tx_data);i++)
    {
        tx_data[i] = i;
    }
    for(i = 0;i < (UART_DATA_LEN / 2);i++)
    {
        check_data0[i*2] = i;
        check_data1[i*2] = UART_DATA_LEN/2 + i;
    }

    dma_block_config_t block1 = {
        .src_address = (uint32_t)0,
        .dst_address = (uint32_t)&dst_data1,
        .p_lli = NULL,
        .CTL_L = DMA_CTLL_INI_EN | DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_BYTE |\
                         DMA_DDATAALIGN_BYTE | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_ENABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)UART_DATA_LEN/2,
        .src_status = 0x0,
        .dst_status = 0x0,
    };

    dma_block_config_t block0 = {
        .src_address = (uint32_t)0,
        .dst_address = (uint32_t)&dst_data0,
        .p_lli = &block1,
        .CTL_L = DMA_CTLL_INI_EN | DMA_DST_INCREMENT | DMA_SRC_NO_CHANGE | DMA_SDATAALIGN_BYTE |\
                         DMA_DDATAALIGN_BYTE | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_ENABLE |\
                         DMA_LLP_SRC_DISABLE | DMA_LLP_DST_ENABLE | DMA_PERIPH_TO_MEMORY,
        .CTL_H = (uint32_t)UART_DATA_LEN/2,
        .src_status = 0x0,
        .dst_status = 0x0,
    };
    dma_sg_llp_config_t sg_llp_config;
    sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_ENABLE;
    sg_llp_config.scatter_config.dst_dsi = (uint32_t)1;
    sg_llp_config.scatter_config.dst_dsc = (uint32_t)1;
    sg_llp_config.gather_config.src_gather_en = DMA_SRC_GATHER_DISABLE;
    sg_llp_config.gather_config.src_sgi = (uint32_t)0;
    sg_llp_config.gather_config.src_sgc = (uint32_t)0;
    sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_DISABLE;
    sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_ENABLE;
    sg_llp_config.llp_config.llp_dst_writeback = 0xB;
    sg_llp_config.llp_config.head_lli = &block0;

    memset(dst_data0,0,sizeof(dst_data0));
    memset(dst_data1,0,sizeof(dst_data1));
    g_rdone = 0;
    g_tdone = 0;
    app_uart_receive_dma_sg_llp(UART_ID, dst_data0, UART_DATA_LEN/2,&sg_llp_config);
    app_uart_dma_transmit_async(UART_ID, tx_data, UART_DATA_LEN);    

    uint8_t delay_count = 100;
    while(g_rdone != 1 && delay_count-- > 0) 
    {
        delay_ms(10);
    }
    delay_ms(10);
    if(g_rdone != 1)
    {
        printf("app_uart_receive_dma_sg_llp_test receive failed\r\n");
    } 
    else
    {
        printf("app_uart_receive_dma_sg_llp_test receive success\r\n");
    }
    if(0 != strncmp((const char*)&check_data0,(const char*)&dst_data0,sizeof(dst_data0))) 
    {
        printf("app_uart_receive_dma_sg_llp_test data0 error\r\n");
    }
    else
    {
        printf("app_uart_receive_dma_sg_llp_test data0 is correct\r\n");
    }
    if(0 != strncmp((const char*)&check_data1,(const char*)&dst_data1,sizeof(dst_data1)))
    {
        printf("app_uart_receive_dma_sg_llp_test data1 error\r\n");
    }
    else
    {
        printf("app_uart_receive_dma_sg_llp_test data1 is correct\r\n");
    }
    if(sg_llp_config.llp_config.llp_dst_writeback != block0.dst_status ||
       sg_llp_config.llp_config.llp_dst_writeback != block1.dst_status ) 
    {
        printf("app_uart_receive_dma_sg_llp_test write back error\r\n");
    }
    else
    {
        printf("app_uart_receive_dma_sg_llp_test write back success\r\n");
    }
    app_log_flush();
    app_uart_deinit(UART_ID);
    app_uart_dma_deinit(UART_ID);
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*          UART APP DMA SG & LLP example.            *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 115200, 8-bits, 1-stopbit, none            *\r\n");
    printf("*                                                    *\r\n");
    printf("*              UART1  <----->   TX to RX             *\r\n");
    printf("*    TX(UART1_TX_PIN)  ----->   RX(UART1_RX_PIN)     *\r\n");
    printf("*    RX(UART1_RX_PIN)  <-----   TX(UART1_TX_PIN)     *\r\n");
    printf("*                                                    *\r\n");
    printf("******************************************************\r\n");

    app_log_flush();

    app_uart_transmit_dma_sg_llp_test();
    app_uart_receive_dma_sg_llp_test();

    printf("UART APP DMA SG & LLP example done!\r\n");

    while(1);
}
