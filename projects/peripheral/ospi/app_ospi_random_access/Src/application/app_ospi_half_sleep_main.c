#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "grx_hal.h"
#include "app_dma.h"
#include "app_graphics_ospi.h"
#include "FreeRTOS.h"
#include "task.h"

#pragma diag_suppress   111

extern const unsigned char s_pattern_data[4096];

#define O_PSRAM_TEST_CLOCK_FREQ             OSPI_CLOCK_FREQ_48MHz
#define O_PSRAM_TEST_DRV_STRNGTH            OSPI_PSRAM_DRV_STR_ONE_FORTH
#define O_PSRAM_TEST_RD_LATENCY             OSPI_PSRAM_RD_LATENCY_3
#define O_PSRAM_TEST_WR_LATENCY             OSPI_PSRAM_WR_LATENCY_3
#define O_PSRAM_TEST_PHY_DELAY              1
#define O_PSRAM_TEST_PREFETCH               0

#define TEST_DMA                            DMA0
#define TEST_DMA_CHANNEL                    DMA_Channel0

static dma_id_t     test_dma_init(uint8_t xfer_width);
static void         test_app_dma_evt_handler(app_dma_evt_type_t type);

static void         ospi_power_on(void);
static void         ospi_power_off(void);
static void         CPU_Access_HalfSleep_test(app_ospi_work_state_e sleep_mode);
static void         DMA_Access_HalfSleep_test(uint32_t xfer_width, app_ospi_work_state_e sleep_mode);
static void         System_Sleep_test(app_ospi_work_state_e sleep_mode);
static void         ospi_init(void);
//static void         load_digcore_vreg(void);
static void         dma_write(uint32_t dest_address, char * data, uint32_t beat_length);
static void         dma_read(uint32_t src_address, char * data, uint32_t beat_length);
static void         cpu_read(uint32_t src_address, char * data, uint32_t length);
static void         cpu_write(uint32_t dest_address, char * data, uint32_t length);

static void dump_data(char * data, uint32_t len);

static dma_id_t s_dma_id;
static volatile bool     is_xfer_cmpt = false;
static volatile bool     is_xfer_err  = false;

__ALIGNED(16) static uint8_t  x_data[4096];


/************************************************************************************
 *                      PUBLIC METHODs
 ************************************************************************************/
int ospi_psram_half_sleep_test(void) {

    /* Please Test One By One*/
    delay_ms(1000);

    //System_Sleep_test(OSPI_STATE_HALF_SLEEP);
    System_Sleep_test(OSPI_STATE_DEEP_SLEEP);

    //CPU_Access_HalfSleep_test(OSPI_STATE_HALF_SLEEP);
    CPU_Access_HalfSleep_test(OSPI_STATE_DEEP_SLEEP);

    //DMA_Access_HalfSleep_test(8,  OSPI_STATE_ACTIVE);
    //DMA_Access_HalfSleep_test(8,  OSPI_STATE_HALF_SLEEP);
    //DMA_Access_HalfSleep_test(16, OSPI_STATE_HALF_SLEEP);
    //DMA_Access_HalfSleep_test(32, OSPI_STATE_HALF_SLEEP);
    DMA_Access_HalfSleep_test(8,  OSPI_STATE_DEEP_SLEEP);
    DMA_Access_HalfSleep_test(16, OSPI_STATE_DEEP_SLEEP);
    DMA_Access_HalfSleep_test(32, OSPI_STATE_DEEP_SLEEP);

    return 0;
}


/************************************************************************************
 *                      STATIC METHODs
 ************************************************************************************/

static void DMA_Access_HalfSleep_test(uint32_t xfer_width, app_ospi_work_state_e sleep_mode) {
    printf("+++ CASE : DMA_Access_HalfSleep_test : %dBit width xfer ...\r\n", xfer_width);

    uint32_t beat_length = 1024;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t failed, total;

    test_dma_init(xfer_width);

    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 2000;
    } else {
        beat_length = 1000;
    }

    while(1) {
        delay_ms(100);
        ospi_init();

        dma_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        dma_read(base_addr, (char*)x_data, beat_length);
        if(memcmp(x_data, s_pattern_data, 4000) != 0) {
            printf(" +++ Please Increase Volatge , Write Fail!\r\n");
        } else {
            printf(" +++ Prepare Write-Data OK !\r\n");
            break;
        }
    }

    total = 0;
    failed = 0;
    uint32_t times = 100;

    while(1) {
        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {
            ospi_power_on();
            delay_us(100);
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
            delay_us(100);
            dma_write(base_addr, (char*)s_pattern_data, beat_length);
        } else {
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
        }

        delay_ms(100);
        times = 500;
#if 1
        while(times --) {
            memset(&x_data[0], 0, 4096);
            dma_read(base_addr, (char*)x_data, beat_length);
            total++;
            if(memcmp(x_data, s_pattern_data, 4000) != 0) {
                failed++;
            }
        }
        printf(" +++ HalfSleep CPU %dBit Access Test, Failed/Total: %d/%d\r\n", xfer_width, failed, total);

        delay_ms(50);
#endif
        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {
            app_graphics_ospi_set_power_state(sleep_mode);
            ospi_power_off();
        } else {
            app_graphics_ospi_set_power_state(sleep_mode);
        }


        delay_ms(100);
    }
}

static void ospi_power_on(void) {
    *((uint32_t volatile *)0xA000A010) = *((uint32_t volatile *)0xA000A010) | 0x0002;
}

static void ospi_power_off(void) {
    *((uint32_t volatile *)0xA000A010) = *((uint32_t volatile *)0xA000A010) & (~0x0002);
}

static void CPU_Access_HalfSleep_test(app_ospi_work_state_e sleep_mode) {
    printf("+++ CASE : CPU_Access_HalfSleep_test ...\r\n");

    uint32_t beat_length = 4096;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t failed, total;

    while(1) {
        delay_ms(100);
        ospi_init();

        cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        cpu_read(base_addr, (char*)x_data, beat_length);
        if(memcmp(x_data, s_pattern_data, beat_length) != 0) {
            printf(" +++ Please Increase Volatge , Write Fail!\r\n");
        } else {
            printf(" +++ Prepare Write-Data OK !\r\n");
            break;
        }
    }

    total = 0;
    failed = 0;
    uint32_t times = 100;

    while(1) {
        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {
            ospi_power_on();
            delay_us(100);
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
            cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        } else {
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
        }

        delay_ms(100);
        times = 100;
#if 1
        while(times --) {
            memset(&x_data[0], 0, 4096);
            cpu_read(base_addr, (char*)x_data, beat_length);
            total++;
            if(memcmp(x_data, s_pattern_data, 4000) != 0) {
                failed++;
            }
        }
        printf(" +++ HalfSleep CPU Access Test, Failed/Total: %d/%d\r\n", failed, total);
#endif
        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {
            app_graphics_ospi_set_power_state(sleep_mode);
            ospi_power_off();
        } else {
            app_graphics_ospi_set_power_state(sleep_mode);
        }


        delay_ms(100);
    }
    printf("\r\n\r\n\r\n");
}


static void System_Sleep_test(app_ospi_work_state_e sleep_mode) {
    printf("+++ CASE : CPU_Access_HalfSleep_test ...\r\n");

    uint32_t beat_length = 4096;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t failed, total;

    while(1) {
        delay_ms(100);
        ospi_init();

        cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        cpu_read(base_addr, (char*)x_data, beat_length);
        if(memcmp(x_data, s_pattern_data, beat_length) != 0) {
            printf(" +++ Please Increase Volatge , Write Fail!\r\n");
        } else {
            printf(" +++ Prepare Write-Data OK !\r\n");
            break;
        }
    }

    total = 0;
    failed = 0;
    uint32_t times = 100;

    while(1) {
        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {
            ospi_power_on();
            delay_us(100);
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
            delay_us(100);
            cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        } else {
            app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
        }

        vTaskDelay(20);
        times = 100;

        while(times --) {
            memset(&x_data[0], 0, 4096);
            cpu_read(base_addr, (char*)x_data, beat_length);
            total++;
            if(memcmp(x_data, s_pattern_data, 4000) != 0) {
                failed++;
                dump_data((char*)x_data, 64);
            }
        }
        printf(" +++ HalfSleep CPU Access Test, Failed/Total: %d/%d \r\n", failed, total);

        if(sleep_mode == OSPI_STATE_DEEP_SLEEP) {

            app_graphics_ospi_set_power_state(sleep_mode);
            ospi_power_off();
        } else {
            app_graphics_ospi_set_power_state(sleep_mode);
        }
        printf("X \r\n");
        vTaskDelay(20);
        printf("eee\r\n");
    }
    printf("\r\n\r\n\r\n");
}

static void dump_data(char * data, uint32_t len) {

    printf("\r\n-----------------------------\r\n");
    for(uint32_t i = 0; i < len / 16; i++) {
        for(uint32_t j = 0; j < 16; j++) {
            printf("%02x ",data[i*16 + j]);
        }
        printf("\r\n");
        delay_ms(1);
    }
    printf("\r\n");
    printf("\r\n");
    delay_ms(10);
}

static void ospi_init(void) {
    app_graphics_ospi_params_t init;

    init.ospi_freq          = O_PSRAM_TEST_CLOCK_FREQ  ;
    init.drv_strength       = O_PSRAM_TEST_DRV_STRNGTH ;
    init.rd_lc              = O_PSRAM_TEST_RD_LATENCY  ;
    init.wr_lc              = O_PSRAM_TEST_WR_LATENCY  ;
    init.phy_delay          = O_PSRAM_TEST_PHY_DELAY   ;
    init.is_read_prefetch   = O_PSRAM_TEST_PREFETCH    ;

    app_graphics_ospi_init(&init);
}


//static void load_digcore_vreg(void) {
//    base_dcdc_vreg = ll_aon_pmu_get_dcdc_vreg();
//    base_ldo_vreg  = ll_aon_pmu_get_dig_ldo_out();
//}

//static void set_digcore_vreg(uint32_t offset) {
//    if(offset >= base_ldo_vreg) {
//        offset = base_ldo_vreg;
//    }

//    ll_aon_pmu_set_dcdc_vreg(base_dcdc_vreg  - offset);
//    ll_aon_pmu_set_dig_ldo_out(base_ldo_vreg + offset);
//}



static void dma_write(uint32_t dest_address, char * data, uint32_t beat_length) {
    is_xfer_cmpt = false;
    is_xfer_err  = false;
    app_dma_start(s_dma_id, (uint32_t)&data[0], dest_address, beat_length);
    while(!is_xfer_cmpt){}
}

static void dma_read(uint32_t src_address, char * data, uint32_t beat_length) {
    is_xfer_cmpt = false;
    is_xfer_err  = false;
    app_dma_start(s_dma_id, src_address, (uint32_t)&data[0], beat_length);
    while(!is_xfer_cmpt){}
}

static void cpu_read(uint32_t src_address, char * data, uint32_t length) {
    memcpy(&data[0], (void*)src_address, length);
}

static void cpu_write(uint32_t dest_address, char * data, uint32_t length) {
    memcpy((void*)dest_address, &data[0],  length);
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
