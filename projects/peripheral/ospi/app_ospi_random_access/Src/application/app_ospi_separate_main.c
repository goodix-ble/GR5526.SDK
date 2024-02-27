#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "grx_hal.h"
#include "app_dma.h"
#include "app_graphics_ospi.h"

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

static void         DMA_Write_Stable_With_Different_Read_Voltage_test(uint32_t xfer_width) ;
static void         DMA_Read_Stabe_With_Different_Write_Voltage_test(uint32_t xfer_width);
static void         CPU_Write_Stable_With_Different_Read_Voltage_test(void) ;
static void         CPU_Read_Stabe_With_Different_Write_Voltage_test(void);
static void         ospi_init(void);
static void         load_digcore_vreg(void);
static void         set_digcore_vreg(uint32_t offset) ;
static void         compare_data(char * src, char * dst, uint32_t length);
static void         dma_write(uint32_t dest_address, char * data, uint32_t beat_length);
static void         dma_read(uint32_t src_address, char * data, uint32_t beat_length);

static void         cpu_write(uint32_t dest_address, char * data, uint32_t length);
static void         cpu_read(uint32_t src_address, char * data, uint32_t length);

static dma_id_t s_dma_id;
static volatile bool     is_xfer_cmpt = false;
static volatile bool     is_xfer_err  = false;
static uint32_t base_dcdc_vreg = 0;
static uint32_t base_ldo_vreg  = 0;
__ALIGNED(16) static uint8_t  x_data[4096];


/************************************************************************************
 *                      PUBLIC METHODs
 ************************************************************************************/
int ospi_psram_separate_test(void) {

    /* Please Test One By One*/

    /* 1. DMA Write Stable, Read  with different */
    DMA_Write_Stable_With_Different_Read_Voltage_test(8);      /* dma xfer in 8  bit*/
    DMA_Write_Stable_With_Different_Read_Voltage_test(16);      /* dma xfer in 16 bit*/
    DMA_Write_Stable_With_Different_Read_Voltage_test(32);      /* dma xfer in 32 bit*/

    /* 2. DMA Read Stable, Write with different */
    DMA_Read_Stabe_With_Different_Write_Voltage_test(8);
    DMA_Read_Stabe_With_Different_Write_Voltage_test(16);
    DMA_Read_Stabe_With_Different_Write_Voltage_test(32);

    // 3. CPU Write Stable, Read  with different */
    CPU_Write_Stable_With_Different_Read_Voltage_test();

    /* 4. CPU Read Stable, Write with different */
    CPU_Read_Stabe_With_Different_Write_Voltage_test();

    return 0;
}


/************************************************************************************
 *                      STATIC METHODs
 ************************************************************************************/

static void DMA_Write_Stable_With_Different_Read_Voltage_test(uint32_t xfer_width) {
    printf("CASE : Write In High Voltage, And Read in low voltage ...\r\n");

    uint32_t beat_length = 1024;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t offset      = 1;

    load_digcore_vreg();
    ospi_init();
    test_dma_init(xfer_width);

    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 2000;
    } else {
        beat_length = 1000;
    }
    static uint32_t fail_array[20] = {0};
    static uint32_t pass_array[20] = {0};

    memset(&fail_array[0], 0, sizeof(uint32_t) * 20);
    memset(&pass_array[0], 0, sizeof(uint32_t) * 20);

    printf("Startup DCDC: %d, IDO: %d \r\n",base_dcdc_vreg, base_ldo_vreg);

    /* STEP 1: Prepare DATA in very high DIGCore Voltage */
    while(1) {
        set_digcore_vreg(12);
        delay_ms(100);
        ospi_init();

        dma_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        dma_read(base_addr, (char*)x_data, beat_length);
        if(memcmp(x_data, s_pattern_data, 4000) != 0) {
            printf(" +++ Please Increase Volatge (%d), Write Fail!\r\n", offset);
        } else {
            printf(" +++ Prepare Write-Data OK !\r\n");
            break;
        }
        offset++;
        offset = offset % 11;
    }


    /* STEP 2: Decrease The DIGCore Voltage to find the errors */
    const uint32_t max_gap = 12;
    offset = 2;
    while(1) {
        set_digcore_vreg(offset);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        dma_read(base_addr, (char*)x_data, beat_length);
        pass_array[offset]++;
        if(memcmp(x_data, s_pattern_data, 4000) != 0) {
            fail_array[offset] = fail_array[offset] + 1;
            printf(" +++ DMA Read %dBit Data with DGCore(+ %d) FAIL ! Total Fail:%d/%d\r\n", xfer_width, offset, fail_array[offset], pass_array[offset]);
            compare_data((char*)s_pattern_data, (char*)x_data, 64);                  /* compare data count, max 4000 */
        } else {
            printf(" +++ DMA Read %dBit Data with DGCore(+ %d) OK ! Total Fail:%d/%d\r\n", xfer_width, offset, fail_array[offset], pass_array[offset]);
        }
        offset ++;
        offset = offset % max_gap;
        if(offset < 2) {
            offset = 2;
        }
    }
    printf("\r\n\r\n\r\n");
}


static void CPU_Write_Stable_With_Different_Read_Voltage_test(void) {
    printf("CASE : Write In High Voltage, And Read in low voltage ...\r\n");

    uint32_t beat_length = 4096;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t offset      = 1;

    load_digcore_vreg();
    ospi_init();

    static uint32_t fail_array[20] = {0};
    static uint32_t pass_array[20] = {0};

    memset(&fail_array[0], 0, sizeof(uint32_t) * 20);
    memset(&pass_array[0], 0, sizeof(uint32_t) * 20);

    printf("Startup DCDC: %d, IDO: %d \r\n",base_dcdc_vreg, base_ldo_vreg);

    /* STEP 1: Prepare DATA in very high DIGCore Voltage */
    while(1) {
        set_digcore_vreg(12);
        delay_ms(100);
        ospi_init();

        cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        cpu_read(base_addr, (char*)x_data, beat_length);
        if(memcmp(x_data, s_pattern_data, beat_length) != 0) {
            printf(" +++ Please Increase Volatge (%d), Write Fail!\r\n", offset);
        } else {
            printf(" +++ Prepare Write-Data OK !\r\n");
            break;
        }
        offset++;
        offset = offset % 11;
    }

    /* STEP 2: Decrease The DIGCore Voltage to find the errors */
    const uint32_t max_gap = 12;
    offset = 2;
    while(1) {
        set_digcore_vreg(offset);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        cpu_read(base_addr, (char*)x_data, beat_length);
        pass_array[offset]++;
        if(memcmp(x_data, s_pattern_data, beat_length) != 0) {
            fail_array[offset] = fail_array[offset] + 1;
            printf(" +++ CPU Read Data with DGCore(+ %d) FAIL ! Total Fail:%d/%d\r\n", offset, fail_array[offset], pass_array[offset]);
            compare_data((char*)s_pattern_data, (char*)x_data, 64);                  /* compare data count, max 4000 */
        } else {
            printf(" +++ CPU Read Data with DGCore(+ %d) OK ! Total Fail:%d/%d\r\n", offset, fail_array[offset], pass_array[offset]);
        }
        offset ++;
        offset = offset % max_gap;
        if(offset < 2) {
            offset = 2;
        }
    }
    printf("\r\n\r\n\r\n");
}


static void ospi_power_off(void) {
    *((uint32_t volatile *)0xA000A010) = *((uint32_t volatile *)0xA000A010) & (~0x0002);
}

static void DMA_Read_Stabe_With_Different_Write_Voltage_test(uint32_t xfer_width) {
    printf("CASE : Write In Different Voltage, And Read in High voltage ...\r\n");

    uint32_t beat_length = 1024;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t offset      = 1;


    static uint32_t fail_array[20] = {0};
    static uint32_t pass_array[20] = {0};

    load_digcore_vreg();
    ospi_init();
    test_dma_init(xfer_width);

    if(8 == xfer_width) {
        beat_length = 4000;
    } else if(16 == xfer_width) {
        beat_length = 2000;
    } else {
        beat_length = 1000;
    }

    memset(&fail_array[0], 0, sizeof(uint32_t) * 20);
    memset(&pass_array[0], 0, sizeof(uint32_t) * 20);

    printf("Startup DCDC: %d, IDO: %d \r\n",base_dcdc_vreg, base_ldo_vreg);

    offset = 0;
    bool ret = 0;
    while(1) {
        ospi_power_off();
        delay_ms(50);
        ospi_init();
        set_digcore_vreg(offset);
        delay_ms(200);
        dma_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(5);
        set_digcore_vreg(13);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        dma_read(base_addr, (char*)x_data, beat_length);
        pass_array[offset]++;
        if(memcmp(x_data, s_pattern_data, 4000) != 0) {
            memset(&x_data[0], 0, 4096);
            dma_read(base_addr, (char*)x_data, beat_length);
            if(memcmp(x_data, s_pattern_data, 4000) != 0) {
                ret = 0;
            } else {
                ret = 1;
            }
        } else {
            ret = 1;
        }

        if(!ret) {
            fail_array[offset] = fail_array[offset] + 1;
            printf(" +++ DMA Read %dBit Data with Write DGCore(+ %d) FAIL ! Total Fail:%d/%d\r\n", xfer_width, offset, fail_array[offset], pass_array[offset]);
            compare_data((char*)s_pattern_data, (char*)x_data, 64);                  /* compare data count, max 4000 */
        } else {
            printf(" +++ DMA Read %dBit Data with Write DGCore(+ %d) OK ! Total Fail:%d/%d\r\n", xfer_width, offset, fail_array[offset], pass_array[offset]);
        }
        memset((void*)base_addr, 0, 4096);    /* clear test area */
        //printf("%08x \r\n", *((uint32_t *)base_addr));
        offset++;
        offset = offset % 13;
    }

    printf("\r\n\r\n\r\n");
}

static void CPU_Read_Stabe_With_Different_Write_Voltage_test(void) {
    printf("CASE : CPU Write In Different Voltage, And Read in High voltage ...\r\n");

    uint32_t beat_length = 4096;
    uint32_t base_addr   = app_graphics_ospi_get_base_address() + 0x1000;
    uint32_t offset      = 1;


    static uint32_t fail_array[20] = {0};
    static uint32_t pass_array[20] = {0};

    load_digcore_vreg();
    ospi_init();

    memset(&fail_array[0], 0, sizeof(uint32_t) * 20);
    memset(&pass_array[0], 0, sizeof(uint32_t) * 20);

    printf("Startup DCDC: %d, IDO: %d \r\n",base_dcdc_vreg, base_ldo_vreg);

    offset = 0;
    bool ret = 0;
    while(1) {
        ospi_power_off();
        delay_ms(50);
        ospi_init();
        set_digcore_vreg(offset);
        delay_ms(200);
        cpu_write(base_addr, (char*)s_pattern_data, beat_length);
        delay_ms(5);
        set_digcore_vreg(13);
        delay_ms(10);
        memset(&x_data[0], 0, 4096);
        cpu_read(base_addr, (char*)x_data, beat_length);
        pass_array[offset]++;
        if(memcmp(x_data, s_pattern_data, 4000) != 0) {
            memset(&x_data[0], 0, 4096);
            cpu_read(base_addr, (char*)x_data, beat_length);
            if(memcmp(x_data, s_pattern_data, 4000) != 0) {
                ret = 0;
            } else {
                ret = 1;
            }
        } else {
            ret = 1;
        }

        if(!ret) {
            fail_array[offset] = fail_array[offset] + 1;
            printf(" +++ CPU Read Data with Write DGCore(+ %d) FAIL ! Total Fail:%d/%d\r\n", offset, fail_array[offset], pass_array[offset]);
            compare_data((char*)s_pattern_data, (char*)x_data, 64);                  /* compare data count, max 4000 */
        } else {
            printf(" +++ CPU Read Data with Write DGCore(+ %d) OK ! Total Fail:%d/%d\r\n", offset, fail_array[offset], pass_array[offset]);
        }
        memset((void*)base_addr, 0, beat_length);    /* clear test area */
        //printf("%08x \r\n", *((uint32_t *)base_addr));
        offset++;
        offset = offset % 13;
    }

    printf("\r\n\r\n\r\n");
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


static void load_digcore_vreg(void) {
    base_dcdc_vreg = ll_aon_pmu_get_dcdc_vreg();
    base_ldo_vreg  = ll_aon_pmu_get_dig_ldo_out();
}

static void set_digcore_vreg(uint32_t offset) {
    if(offset >= base_ldo_vreg) {
        offset = base_ldo_vreg;
    }

    ll_aon_pmu_set_dcdc_vreg(base_dcdc_vreg  - offset);
    ll_aon_pmu_set_dig_ldo_out(base_ldo_vreg + offset);
}

static void compare_data(char * src, char * dst, uint32_t length) {
    printf("\r\n\r\n");
    printf("NO.   EXP    ERR \r\n");
    printf("------------------\r\n");
    for(uint32_t i = 0 ; i< length; i++) {
        if(src[i] != dst[i]) {
            printf("%03d    %02x    %02x\r\n",i, src[i], dst[i]);
            delay_ms(1);
        }
    }
    printf("\r\n\r\n");
    delay_ms(10);
}


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

static void cpu_write(uint32_t dest_address, char * data, uint32_t length) {
    memcpy((void*)dest_address, data, length);
}

static void cpu_read(uint32_t src_address, char * data, uint32_t length) {
    memcpy(data, (void*)src_address, length);
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


