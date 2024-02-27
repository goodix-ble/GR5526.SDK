#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "app_dma.h"
#include "app_graphics_ospi.h"

#include "FreeRTOS.h"
#include "task.h"

#define TEST_TIMES          100

#define O_PSRAM_TEST_CLOCK_FREQ             OSPI_CLOCK_FREQ_48MHz
#define O_PSRAM_TEST_DRV_STRNGTH            OSPI_PSRAM_DRV_STR_ONE_FORTH
#define O_PSRAM_TEST_RD_LATENCY             OSPI_PSRAM_RD_LATENCY_3
#define O_PSRAM_TEST_WR_LATENCY             OSPI_PSRAM_WR_LATENCY_3
#define O_PSRAM_TEST_PHY_DELAY              1
#define O_PSRAM_TEST_PREFETCH               0

#define TEST_DMA_XFER_WIDTH                 8

static void     ospi_test_dma_pressure_test(void);

static uint32_t test_mcu_access(uint8_t mode, uint32_t times);
static uint32_t test_dma0_access(uint32_t xfer_width, uint32_t times);
static dma_id_t test_dma0_init(uint8_t xfer_width);
static void     test_app_dma0_evt_handler(app_dma_evt_type_t type);
static dma_id_t test_dma1_init(uint8_t xfer_width);
static void     test_app_dma1_evt_handler(app_dma_evt_type_t type);
static uint32_t test_dma1_access(uint32_t xfer_width, uint32_t times);
    static uint32_t test_dmax_access(uint32_t xfer_width, uint32_t times);
static dma_id_t s_dma0_id;
static dma_id_t s_dma1_id;
static volatile bool     is_dma0_xfer_cmpt = false;
static volatile bool     is_dma0_xfer_err  = false;

static volatile bool     is_dma1_xfer_cmpt = false;
static volatile bool     is_dma1_xfer_err  = false;


/************************************************************************************
 *                      PUBLIC METHODs
 ************************************************************************************/
int dma0_pressure_test(void ) {


    /* 2. DMA Random Access */
    test_dma0_init(32);
    test_dma0_access(32, 0xFFFFFFF0);

    while(1){}

    return 0;
}

int dma1_pressure_test(void ) {


    /* 2. DMA Random Access */
    test_dma1_init(32);
    test_dma1_access(32, 0xFFFFFFF0);

    while(1){}

    return 0;
}


void dmax_init(void) {
    test_dma0_init(32);
    test_dma1_init(32);
}
int dmax_pressure_test(void ) {
    /* 2. DMA Random Access */

    test_dmax_access(1, 0xFFFFFFF0);

    while(1){}

    return 0;
}


uint8_t s_dma0_data[4096*4];

static uint32_t test_dma0_access(uint32_t xfer_width, uint32_t times) {
    uint32_t base_addr = 0x00200000;//app_graphics_ospi_get_base_address() - CONCAT_RAM_SIZE;
    uint32_t i;
    const uint32_t delta = 0x100;
    uint32_t beat_length = 0;
    uint32_t actual_addr = 0, offset = 0;
    offset = 0;


    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 4000;
    } else {
        beat_length = 4000;
    }

    for(i = 0; i < times ; i++) {
        actual_addr = base_addr + (i * delta) % 0x600000;

        is_dma0_xfer_cmpt = false;
        is_dma0_xfer_err  = false;
        app_dma_start(s_dma0_id, (uint32_t)base_addr, (uint32_t)&s_dma0_data[0], beat_length);
        while(!is_dma0_xfer_cmpt){
            vTaskDelay(1);
        }

        if(i % 1000 == 0) {
            printf("+++ DMA0 %dBit ACCESS %d...\r\n", xfer_width, i);
        }
    }

    return 0;
}


uint8_t s_dma1_data[4096*4];

static uint32_t test_dma1_access(uint32_t xfer_width, uint32_t times) {
    uint32_t base_addr = 0x00200000;//app_graphics_ospi_get_base_address() - CONCAT_RAM_SIZE;
    uint32_t i;
    const uint32_t delta = 0x100;
    uint32_t beat_length = 0;
    uint32_t actual_addr = 0, offset = 0;
    offset = 0;


    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 4000;
    } else {
        beat_length = 4000;
    }

    for(i = 0; i < times ; i++) {
        actual_addr = base_addr + (i * delta) % 0x600000;

        is_dma1_xfer_cmpt = false;
        is_dma1_xfer_err  = false;
        app_dma_start(s_dma1_id, (uint32_t)base_addr, (uint32_t)&s_dma1_data[0], beat_length);
        while(!is_dma1_xfer_cmpt){
            vTaskDelay(1);
        }

        if(i % 1000 == 0) {
            printf("+++ DMA1 %dBit ACCESS %d...\r\n", xfer_width, i);
        }
    }

    return 0;
}


static uint32_t test_dmax_access(uint32_t xfer_width, uint32_t times) {
    uint32_t base_addr = 0x00200000;//app_graphics_ospi_get_base_address() - CONCAT_RAM_SIZE;
    uint32_t i;
    uint32_t beat_length = 0;
    uint32_t actual_addr = 0, offset = 0;

    beat_length = 4000;

    for(i = 0; i < times ; i++) {
        actual_addr = base_addr;

        is_dma0_xfer_cmpt = false;
        is_dma1_xfer_cmpt  = false;
        app_dma_start(s_dma0_id, (uint32_t)base_addr, (uint32_t)&s_dma0_data[0], beat_length);
        //app_dma_start(s_dma1_id, (uint32_t)base_addr + 0x10000, (uint32_t)&s_dma1_data[0], beat_length);
        while(!is_dma0_xfer_cmpt ){

        }
    }

    return 0;
}


static dma_id_t test_dma0_init(uint8_t xfer_width) {
    app_dma_params_t dma_params = {0};

    dma_params.p_instance                 = DMA0;
    dma_params.channel_number             = DMA_Channel0;
    dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;
    dma_params.init.src_request           = DMA0_REQUEST_MEM;
    dma_params.init.dst_request           = DMA0_REQUEST_MEM;

    if(xfer_width == 8) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    } else if(xfer_width == 16) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    } else if(xfer_width == 32) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }

    s_dma0_id = app_dma_init(&dma_params, test_app_dma0_evt_handler);
    if (s_dma0_id < 0)
    {
        printf("DMA0 Init Failed!\r\n");
    }

    return s_dma0_id;
}

static void test_app_dma0_evt_handler(app_dma_evt_type_t type) {
    switch(type)  {
        case APP_DMA_EVT_TFR:
        case APP_DMA_EVT_BLK:
            is_dma0_xfer_cmpt = true;
            is_dma0_xfer_err  = false;
            break;
        case APP_DMA_EVT_ERROR:
            is_dma0_xfer_cmpt = true;
            is_dma0_xfer_err  = true;
            printf("DMA Xfer Err...\r\n");
            break;
    }

    return;
}

static dma_id_t test_dma1_init(uint8_t xfer_width) {
    app_dma_params_t dma_params = {0};

    dma_params.p_instance                 = DMA1;
    dma_params.channel_number             = DMA_Channel0;
    dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;
    dma_params.init.src_request           = DMA1_REQUEST_MEM;
    dma_params.init.dst_request           = DMA1_REQUEST_MEM;

    if(xfer_width == 8) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    } else if(xfer_width == 16) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    } else if(xfer_width == 32) {
        dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }

    s_dma1_id = app_dma_init(&dma_params, test_app_dma1_evt_handler);
    if (s_dma1_id < 0)
    {
        printf("DMA1 Init Failed!\r\n");
    }

    return s_dma1_id;
}

static void test_app_dma1_evt_handler(app_dma_evt_type_t type) {
    switch(type)  {
        case APP_DMA_EVT_TFR:
        case APP_DMA_EVT_BLK:
            is_dma1_xfer_cmpt = true;
            is_dma1_xfer_err  = false;
            break;
        case APP_DMA_EVT_ERROR:
            is_dma1_xfer_cmpt = true;
            is_dma1_xfer_err  = true;
            printf("DMA Xfer Err...\r\n");
            break;
    }

    return;
}
