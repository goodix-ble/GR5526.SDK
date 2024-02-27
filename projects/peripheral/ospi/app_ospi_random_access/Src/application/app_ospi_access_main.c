#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "app_dma.h"
#include "app_graphics_ospi.h"

#define TEST_TIMES          5u
#define CONCAT_RAM_SIZE     5000u
#define TEST_DMA            DMA0
#define TEST_DMA_CHANNEL    DMA_Channel0

static void     ospi_test_mcu_access(void);
static void     ospi_test_dma_access(void);

static bool     test_mcu_access(uint32_t times);
static void     tes_mcu_test_log(uint32_t cnt, app_graphics_ospi_params_t param, bool ret);
static dma_id_t test_dma_init(uint8_t xfer_width);
static void     test_app_dma_evt_handler(app_dma_evt_type_t type);
static bool     test_dma_access(uint32_t times);
static void     tes_dma_test_log(uint32_t xfer_width, uint32_t cnt, app_graphics_ospi_params_t param, bool ret);

static dma_id_t s_dma_id;
static volatile bool is_xfer_cmpt = false;
static volatile bool is_xfer_err  = false;

static volatile uint32_t fail_times = 0;
static volatile uint32_t pass_times = 0;

extern const unsigned char ospi_test_raw_data[16384];

/************************************************************************************
 *                      PUBLIC METHODs
 ************************************************************************************/
int ospi_psram_access_test(void) {

    /* 1. MCU Random Access */
    ospi_test_mcu_access();

    /* 2. DMA Random Access */
    ospi_test_dma_access();

    return 0;
}


/************************************************************************************
 *                      STATIC METHODs
 ************************************************************************************/
static void ospi_test_mcu_access(void) {
    app_ospi_clock_freq_e ospi_freq[4] = {  OSPI_CLOCK_FREQ_48MHz,
                                            OSPI_CLOCK_FREQ_32MHz,
                                            OSPI_CLOCK_FREQ_24MHz,
                                            OSPI_CLOCK_FREQ_16MHz,
                                         };

    app_ospi_psram_drv_strength_e   drv_strength[4] = {
                                            OSPI_PSRAM_DRV_STR_ONE_EIGHTH,
                                            OSPI_PSRAM_DRV_STR_ONE_FORTH ,
                                            OSPI_PSRAM_DRV_STR_HALF      ,
                                            OSPI_PSRAM_DRV_STR_FULL      ,
                                         };

    app_ospi_psram_rd_latency_e     rd_lc[5] = {
                                            OSPI_PSRAM_RD_LATENCY_3,
                                            OSPI_PSRAM_RD_LATENCY_4,
                                            OSPI_PSRAM_RD_LATENCY_5,
                                            OSPI_PSRAM_RD_LATENCY_6,
                                            OSPI_PSRAM_RD_LATENCY_7,
                                        };

    app_ospi_psram_wr_latency_e     wr_lc[5] = {
                                            OSPI_PSRAM_WR_LATENCY_3,
                                            OSPI_PSRAM_WR_LATENCY_4,
                                            OSPI_PSRAM_WR_LATENCY_5,
                                            OSPI_PSRAM_WR_LATENCY_6,
                                            OSPI_PSRAM_WR_LATENCY_7,
                                        };

    uint32_t                        phy_delay[3] = {1, 2, 3};
    uint8_t                         is_read_prefetch[2] = {0, 1};
    uint32_t a,b,c,d,e,f, cnt;
    bool ret;
    app_graphics_ospi_params_t init;

    cnt  = 0;
    for(f = 0; f < 2; f++) {
        for(e = 0; e < 1; e++) {
            for(d = 0; d < 5; d++) {
                for(c = 0; c < 5; c++) {
                    for(b = 0; b < 4; b++) {
                        for(a = 0; a < 4; a++) {
                            init.ospi_freq          = ospi_freq[a];
                            init.drv_strength       = drv_strength[b];
                            init.rd_lc              = rd_lc[c];
                            init.wr_lc              = wr_lc[d];
                            init.phy_delay          = phy_delay[e];
                            init.is_read_prefetch   = is_read_prefetch[f];
                            app_graphics_ospi_init(&init);

                            ret = test_mcu_access(TEST_TIMES);
                            tes_mcu_test_log(cnt, init, ret);

                            app_graphics_ospi_deinit();
                            cnt++;
                        }
                    }
                }
            }
        }
    }

    printf("\r\n\r\n");

    return;
}

static void ospi_test_dma_access(void) {

    app_ospi_clock_freq_e ospi_freq[4] = {  OSPI_CLOCK_FREQ_48MHz,
                                            OSPI_CLOCK_FREQ_32MHz,
                                            OSPI_CLOCK_FREQ_24MHz,
                                            OSPI_CLOCK_FREQ_16MHz,
                                         };

    app_ospi_psram_drv_strength_e   drv_strength[4] = {
                                            OSPI_PSRAM_DRV_STR_ONE_EIGHTH,
                                            OSPI_PSRAM_DRV_STR_ONE_FORTH ,
                                            OSPI_PSRAM_DRV_STR_HALF      ,
                                            OSPI_PSRAM_DRV_STR_FULL      ,
                                         };

    app_ospi_psram_rd_latency_e     rd_lc[3] = {
                                            OSPI_PSRAM_RD_LATENCY_3,
                                            OSPI_PSRAM_RD_LATENCY_4,
                                            OSPI_PSRAM_RD_LATENCY_5,
                                        };

    app_ospi_psram_wr_latency_e     wr_lc[3] = {
                                            OSPI_PSRAM_WR_LATENCY_3,
                                            OSPI_PSRAM_WR_LATENCY_4,
                                            OSPI_PSRAM_WR_LATENCY_5,
                                        };

    uint32_t                        phy_delay[3] = {1, 2, 3};
    uint8_t                         is_read_prefetch[2] = {0, 1};
    uint32_t                        xfer_width[3] = {8, 16, 32};

    uint32_t a,b,c,d,e,f,g, cnt;
    bool ret;
    app_graphics_ospi_params_t init;

    cnt  = 0;
    for(g = 0; g < 2; g++) {
        for(f = 0; f < 2; f++) {
            for(e = 0; e < 1; e++) {
                for(d = 0; d < 3; d++) {
                    for(c = 0; c < 3; c++) {
                        for(b = 0; b < 4; b++) {
                            for(a = 0; a < 4; a++) {
                                init.ospi_freq          = ospi_freq[a];
                                init.drv_strength       = drv_strength[b];
                                init.rd_lc              = rd_lc[c];
                                init.wr_lc              = wr_lc[d];
                                init.phy_delay          = phy_delay[e];
                                init.is_read_prefetch   = is_read_prefetch[f];
                                app_graphics_ospi_init(&init);
                                test_dma_init(xfer_width[g]);

                                ret = test_dma_access(TEST_TIMES);
                                tes_dma_test_log(xfer_width[g], cnt, init, ret);

                                app_dma_deinit(s_dma_id);
                                s_dma_id = -1;
                                app_graphics_ospi_deinit();
                                cnt++;
                            }
                        }
                    }
                }
            }
        }
    }

    printf("\r\n\r\n");

    return;

}

static bool test_mcu_access(uint32_t times) {
    uint32_t base_addr = app_graphics_ospi_get_base_address();
    uint32_t i;
    const uint32_t delta = 0x401;
    uint32_t actual_addr = 0;

    fail_times = 0;
    for(i = 0; i < times; i++) {
        actual_addr = base_addr + (i * delta) % 0x600000;

        memcpy((void*)actual_addr, &ospi_test_raw_data[0], sizeof(ospi_test_raw_data));
        if(memcmp((void*)actual_addr, &ospi_test_raw_data[0], sizeof(ospi_test_raw_data)) != 0){
            fail_times ++;
            //return false;
        }
    }

    pass_times = times - fail_times;
    return true;
}

static void tes_mcu_test_log(uint32_t cnt, app_graphics_ospi_params_t param, bool ret) {
    char * ospi_freq_str[4] = {"48MHz", "32MHz", "24MHz", "16MHz"};
    char * drv_strength[4] = {"1.0", "0.5", "0.25", "0.125"};
    char * rd_lc[5] = {"3", "4", "5", "6", "7"};
    char * wr_lc[5] = {"3", "4", "5", "6", "7"};

    printf("+++ (%04d) MCU.ACCESS [%s] [Drv:%5s] [RDL:%s] [WRL:%s] [PHY:%d] [PRE:%d], FAIL: %d/%d \r\n",
            cnt, ospi_freq_str[param.ospi_freq], drv_strength[param.drv_strength],
            rd_lc[param.rd_lc], wr_lc[param.wr_lc], param.phy_delay, param.is_read_prefetch,
            fail_times, (fail_times + pass_times)
    );
}


static uint8_t x_data[4096];

static bool test_dma_access(uint32_t times) {
    uint32_t base_addr = app_graphics_ospi_get_base_address();
    uint32_t i;
    const uint32_t delta = 0x400;
    uint32_t actual_addr = 0;

    fail_times = 0;
    for(i = 0; i < times; i++) {
        actual_addr = base_addr + (i * delta) % 0x600000;

        is_xfer_cmpt = false;
        is_xfer_err  = false;
        app_dma_start(s_dma_id, (uint32_t)&ospi_test_raw_data[0], actual_addr, 4000);
        while(!is_xfer_cmpt){}

        memset(&x_data[0], 0, 4096);
        is_xfer_cmpt = false;
        is_xfer_err  = false;
        app_dma_start(s_dma_id, actual_addr, (uint32_t)&x_data[0], 4000);
        while(!is_xfer_cmpt){}

        is_xfer_err = is_xfer_err;
        if(memcmp((void*)&x_data[0], &ospi_test_raw_data[0], 4000) != 0){
            fail_times ++;
        }
    }

    pass_times = times - fail_times;
    return true;
}

static void tes_dma_test_log(uint32_t xfer_width, uint32_t cnt, app_graphics_ospi_params_t param, bool ret) {
    char * ospi_freq_str[4] = {"48MHz", "32MHz", "24MHz", "16MHz"};
    char * drv_strength[4] = {"1.0", "0.5", "0.25", "0.125"};
    char * rd_lc[5] = {"3", "4", "5", "6", "7"};
    char * wr_lc[5] = {"3", "4", "5", "6", "7"};

    printf("+++ (%04d) DMA.ACCESS [%d] [%s] [Drv:%5s] [RDL:%s] [WRL:%s] [PHY:%d] [PRE:%d], FAIL: %d/%d \r\n",
            cnt, xfer_width, ospi_freq_str[param.ospi_freq], drv_strength[param.drv_strength],
            rd_lc[param.rd_lc], wr_lc[param.wr_lc], param.phy_delay, param.is_read_prefetch,
            fail_times, (fail_times + pass_times)
    );
}

static dma_id_t test_dma_init(uint8_t xfer_width) {
    app_dma_params_t dma_params = {0};

    dma_params.p_instance                 = TEST_DMA;
    dma_params.channel_number             = TEST_DMA_CHANNEL;
    dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;
    dma_params.init.src_request           = (TEST_DMA == DMA0) ? DMA0_REQUEST_MEM : DMA1_REQUEST_MEM;
    dma_params.init.dst_request           = (TEST_DMA == DMA0) ? DMA0_REQUEST_MEM : DMA1_REQUEST_MEM;

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

    s_dma_id = app_dma_init(&dma_params, test_app_dma_evt_handler);
    if (s_dma_id < 0)
    {
        printf("DMA Init Failed!\r\n");
    }

    return s_dma_id;
}

static void test_app_dma_evt_handler(app_dma_evt_type_t type) {
    switch(type)  {
        case APP_DMA_EVT_TFR:
        case APP_DMA_EVT_BLK:
            is_xfer_cmpt = true;
            is_xfer_err  = false;
            break;
        case APP_DMA_EVT_ERROR:
            is_xfer_cmpt = true;
            is_xfer_err  = true;
            printf("DMA Xfer Err...\r\n");
            break;
    }

    return;
}
