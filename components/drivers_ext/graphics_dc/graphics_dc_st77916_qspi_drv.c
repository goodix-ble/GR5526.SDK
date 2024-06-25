#include "graphics_dc_st77916_qspi_drv.h"
#include "app_graphics_dc.h"
#include "app_io.h"
#include "gr55xx_ll_gpio.h"
#include "gr55xx_ll_aon_gpio.h"
#include "gr55xx_sys_sdk.h"

#ifdef USE_OSAL
#include "osal.h"
#else // USE_OSAL
#include "FreeRTOS.h"
#include "semphr.h"
#endif // USE_OSAL

#include "FreeRTOS.h"

#define DISPLAY_RESET_IO_TYPE APP_IO_TYPE_AON
#define DISPLAY_RESET_IO_PIN  APP_IO_PIN_6
#define DISPLAY_RESET_IO_MUX  APP_IO_MUX

#define DISPLAY_BL_IO_TYPE APP_IO_TYPE_GPIOC
#define DISPLAY_BL_IO_PIN  APP_IO_PIN_1
#define DISPLAY_BL_IO_MUX  APP_IO_MUX

#define DISPLAY_TE_IO_TYPE APP_IO_TYPE_AON
#define DISPLAY_TE_IO_PIN  APP_IO_PIN_5
#define DISPLAY_TE_IO_MUX  APP_IO_MUX

#define ST77916_INST_WR_I1A1D1 (0x02)
#define ST77916_INST_WR_I1A1D2 (0xA2)
#define ST77916_INST_WR_I1A1D4 (0x32)
#define ST77916_INST_WR_I1A4D4 (0x38)

#define ST77916_INST_RD (0x0B)

#define ST77916_TE_WAIT_TIMEOUT_MS (100)

static void dc_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t len);
static void st77916_init_sequence(void);
static void display_te_set_enable(bool enabled);
static void display_te_evt_callback(app_io_evt_t *p_evt);

static bool s_te_valid = true;
#ifdef USE_OSAL
static osal_sema_handle_t s_te_sem;
#else // USE_OSAL
static SemaphoreHandle_t s_te_sem;
#endif // USE_OSAL

/**
 * Global Functions
*/

void graphics_dc_st77916_init(uint16_t screen_w, uint16_t screen_h)
{
    // Initialize Display Controller
    app_graphics_dc_params_t dc_params = {
        .mspi_mode = GDC_MODE_QSPI,
        .clock_freq = GDC_CLOCK_FREQ_48MHz,
        .clock_mode = GDC_CLOCK_MODE_0,
        .tcsu_cycle = GDC_TCSU_CYCLE_1,
        .layer_mode = GDC_ONE_LAYER_MODE,
        .mipicfg_format = GDC_MIPICFG_QSPI_RGB565_OPT0, // ST77916 Only support RGB565 and RGB666
        .resolution_x = screen_w,
        .resolution_y = screen_h,
        .pins_cfg = {
            .csn = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .clk = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io0 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io1 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io2 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io3 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .dcx = {
                .pull = APP_IO_PULLUP,
                .enable = DISABLE,
            },
        },
    };

    graphics_dc_init(&dc_params, NULL);

    // Create semaphore for TE Sync
#ifdef USE_OSAL
    osal_sema_binary_create(&s_te_sem);
#else
    s_te_sem = xSemaphoreCreateBinary();
#endif // USE_OSAL

    // Initialize I/O
    app_io_init_t io_init;

    // Reset pin
    io_init.pin = DISPLAY_RESET_IO_PIN;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pull = APP_IO_NOPULL;
    io_init.mux = DISPLAY_RESET_IO_MUX;
    app_io_init(DISPLAY_RESET_IO_TYPE, &io_init);
    app_io_write_pin(DISPLAY_RESET_IO_TYPE, DISPLAY_RESET_IO_PIN, APP_IO_PIN_SET);

    // TE
    io_init.pin = DISPLAY_TE_IO_PIN;
    io_init.mode = APP_IO_MODE_IT_FALLING;
    io_init.pull = APP_IO_PULLUP;
    io_init.mux = DISPLAY_TE_IO_MUX;
    app_io_event_register_cb(DISPLAY_TE_IO_TYPE, &io_init, display_te_evt_callback, NULL);
    display_te_set_enable(false);

    // perform ST77916 initialization sequence
    st77916_init_sequence();

    // Enable Blacklight finally
    io_init.pin = DISPLAY_BL_IO_PIN;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pull = APP_IO_NOPULL;
    io_init.mux = DISPLAY_BL_IO_MUX;
    app_io_init(DISPLAY_BL_IO_TYPE, &io_init);
    app_io_write_pin(DISPLAY_BL_IO_TYPE, DISPLAY_BL_IO_PIN, APP_IO_PIN_SET);
}

void graphics_dc_st77916_deinit(void)
{
    graphics_dc_deinit();
}

void graphics_dc_st77916_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint8_t data_2a[4] = {
        (x1 & 0xFF00) >> 8,
        x1 & 0x00FF,
        (x2 & 0xFF00) >> 8,
        x2 & 0x00FF,
    };

    uint8_t data_2b[4] = {
        (y1 & 0xFF00) >> 8,
        y1 & 0x00FF,
        (y2 & 0xFF00) >> 8,
        y2 & 0x00FF,
    };

    dc_spi_send_sync(0x2A, data_2a, 4);
    dc_spi_send_sync(0x2B, data_2b, 4);
}

void graphics_dc_st77916_flush(void *buf, uint32_t buf_format, uint16_t w, uint16_t h)
{
    app_graphics_dc_cmd_t dc_cmd = {
        .command = ST77916_INST_WR_I1A4D4,
        .address = 0x002C00,
        .address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT,
        .frame_timing = GDC_QSPI_FRAME_TIMING_1,
    };

    app_graphics_dc_framelayer_t dc_layer = {
        .frame_baseaddr = buf,
        .resolution_x = w,
        .resolution_y = h,
        .row_stride = -1,
        .start_x = 0,
        .start_y = 0,
        .size_x = w,
        .size_y = h,
        .alpha = 0,
        .blendmode = HAL_GDC_BL_SRC,
        .data_format = (graphics_dc_data_format_e)buf_format,
    };

    app_graphics_dc_send_single_frame(GRAPHICS_DC_LAYER_0, &dc_layer, &dc_cmd, GDC_ACCESS_TYPE_ASYNC);
}

void graphics_dc_st77916_set_on(bool on)
{
    app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    if (on)
    {
        printf("Send 29h\n");
        dc_spi_send_sync(0x29, NULL, 0);
        printf("Backlight on\n");
        app_io_write_pin(DISPLAY_BL_IO_TYPE, DISPLAY_BL_IO_PIN, APP_IO_PIN_SET);
    }
    else
    {
        printf("Send 28h\n");
        dc_spi_send_sync(0x28, NULL, 0);
        printf("Backlight off\n");
        uint16_t ret = app_io_write_pin(DISPLAY_BL_IO_TYPE, DISPLAY_BL_IO_PIN, APP_IO_PIN_RESET);
        printf("ret = %d\n", ret);
        portENABLE_INTERRUPTS();
    }
}

void graphics_dc_st77916_wait_te(void)
{
    if (s_te_valid)
    {
        display_te_set_enable(true);
#ifdef USE_OSAL
        int32_t ret = osal_sema_take(s_te_sem, ST77916_TE_WAIT_TIMEOUT_MS);
        if (OSAL_SUCCESS != ret)
        {
            // Once timeout, never wait for TE again
            s_te_valid = false;
        }
#else // USE_OSAL
        BaseType_t ret = xSemaphoreTake(s_te_sem, ST77916_TE_WAIT_TIMEOUT_MS);
        if (pdPASS != ret)
        {
            // Once timeout, never wait for TE again
            s_te_valid = false;
        }
#endif // USE_OSAL
        display_te_set_enable(false);
    }
}

void graphics_dc_st77916_wait_ready(void)
{
    // Modify DC power state will wait for ongoing process internally
    app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
}

void graphics_dc_st77916_sleep(void)
{
    // TODO: ST77916 deep sleep & AoD
}

void graphics_dc_st77916_wakeup(void)
{
    // TODO: ST77916 Wakeup
}

void graphics_dc_st77916_set_brightness(uint32_t percentage)
{
    // TODO: ST77916 Brightness
}

/**
 * Static Functions
*/

static void dc_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t len)
{
    if ((cmd & 0xFF) == cmd)
    {
        cmd <<= 8;
    }
    app_graphics_dc_spi_send(ST77916_INST_WR_I1A1D1, cmd, data, len);
}

static void st77916_init_sequence(void)
{
    // Reset
    delay_ms(10);
    app_io_write_pin(DISPLAY_RESET_IO_TYPE, DISPLAY_RESET_IO_PIN, APP_IO_PIN_RESET);
    delay_ms(10);
    app_io_write_pin(DISPLAY_RESET_IO_TYPE, DISPLAY_RESET_IO_PIN, APP_IO_PIN_SET);
    delay_ms(10);

    // Init command sequence
    uint8_t data[32];
    uint16_t datalen = 0;

    datalen = 0;
    data[datalen++] = 0x08;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x08;
    dc_spi_send_sync(0xF2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x51;
    dc_spi_send_sync(0x9B, data, datalen);

    datalen = 0;
    data[datalen++] = 0x53;
    dc_spi_send_sync(0x86, data, datalen);

    datalen = 0;
    data[datalen++] = 0x80;
    dc_spi_send_sync(0xF2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0xF1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x55;
    dc_spi_send_sync(0xB0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x1E;
    dc_spi_send_sync(0xB1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x3B;
    dc_spi_send_sync(0xB2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x06;
    dc_spi_send_sync(0xB4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x24;
    dc_spi_send_sync(0xB5, data, datalen);

    datalen = 0;
    data[datalen++] = 0xA5;
    dc_spi_send_sync(0xB6, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xB7, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xBA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x08;
    dc_spi_send_sync(0xBB, data, datalen);

    datalen = 0;
    data[datalen++] = 0x08;
    dc_spi_send_sync(0xBC, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xBD, data, datalen);

    datalen = 0;
    data[datalen++] = 0x80;
    dc_spi_send_sync(0xC0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xC1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x37;
    dc_spi_send_sync(0xC2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x80;
    dc_spi_send_sync(0xC3, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xC4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x37;
    dc_spi_send_sync(0xC5, data, datalen);

    datalen = 0;
    data[datalen++] = 0xA9;
    dc_spi_send_sync(0xC6, data, datalen);

    datalen = 0;
    data[datalen++] = 0x41;
    dc_spi_send_sync(0xC7, data, datalen);

    datalen = 0;
    data[datalen++] = 0x51;
    dc_spi_send_sync(0xC8, data, datalen);

    datalen = 0;
    data[datalen++] = 0xA9;
    dc_spi_send_sync(0xC9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x41;
    dc_spi_send_sync(0xCA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x51;
    dc_spi_send_sync(0xCB, data, datalen);

    datalen = 0;
    data[datalen++] = 0x91;
    dc_spi_send_sync(0xD0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x68;
    dc_spi_send_sync(0xD1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x69;
    dc_spi_send_sync(0xD2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    data[datalen++] = 0xA5;
    dc_spi_send_sync(0xF5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x3B;
    dc_spi_send_sync(0xDD, data, datalen);

    datalen = 0;
    data[datalen++] = 0x3B;
    dc_spi_send_sync(0xDE, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xF1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0xf0;
    data[datalen++] = 0x0B;
    data[datalen++] = 0x12;
    data[datalen++] = 0x0B;
    data[datalen++] = 0x0A;
    data[datalen++] = 0x06;
    data[datalen++] = 0x39;
    data[datalen++] = 0x43;
    data[datalen++] = 0x4F;
    data[datalen++] = 0x07;
    data[datalen++] = 0x14;
    data[datalen++] = 0x14;
    data[datalen++] = 0x2f;
    data[datalen++] = 0x34;
    dc_spi_send_sync(0xe0, data, datalen);

    datalen = 0;
    data[datalen++] = 0xf0;
    data[datalen++] = 0x0B;
    data[datalen++] = 0x11;
    data[datalen++] = 0x0A;
    data[datalen++] = 0x09;
    data[datalen++] = 0x05;
    data[datalen++] = 0x32;
    data[datalen++] = 0x33;
    data[datalen++] = 0x48;
    data[datalen++] = 0x07;
    data[datalen++] = 0x13;
    data[datalen++] = 0x13;
    data[datalen++] = 0x2C;
    data[datalen++] = 0x33;
    dc_spi_send_sync(0xe1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xF3, data, datalen);

    datalen = 0;
    data[datalen++] = 0x0A;
    dc_spi_send_sync(0xE0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xE1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xE2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xE3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE0;
    dc_spi_send_sync(0xE4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x06;
    dc_spi_send_sync(0xE5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x21;
    dc_spi_send_sync(0xE6, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xE7, data, datalen);

    datalen = 0;
    data[datalen++] = 0x05;
    dc_spi_send_sync(0xE8, data, datalen);

    datalen = 0;
    data[datalen++] = 0xF2;
    dc_spi_send_sync(0xE9, data, datalen);

    datalen = 0;
    data[datalen++] = 0xDF;
    dc_spi_send_sync(0xEA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x80;
    dc_spi_send_sync(0xEB, data, datalen);

    datalen = 0;
    data[datalen++] = 0x20;
    dc_spi_send_sync(0xEC, data, datalen);

    datalen = 0;
    data[datalen++] = 0x14;
    dc_spi_send_sync(0xED, data, datalen);

    datalen = 0;
    data[datalen++] = 0xFF;
    dc_spi_send_sync(0xEE, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xEF, data, datalen);

    datalen = 0;
    data[datalen++] = 0xFF;
    dc_spi_send_sync(0xF8, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xF9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xFA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x30;
    dc_spi_send_sync(0xFB, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xFC, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xFD, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xFE, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xFF, data, datalen);

    datalen = 0;
    data[datalen++] = 0x42;
    dc_spi_send_sync(0x60, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE0;
    dc_spi_send_sync(0x61, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x62, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x63, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x64, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x65, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x66, data, datalen);

    datalen = 0;
    data[datalen++] = 0x03;
    dc_spi_send_sync(0x67, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x68, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x69, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x6A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x6B, data, datalen);

    datalen = 0;
    data[datalen++] = 0x42;
    dc_spi_send_sync(0x70, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE0;
    dc_spi_send_sync(0x71, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x72, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x73, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x74, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x75, data, datalen);

    datalen = 0;
    data[datalen++] = 0x40;
    dc_spi_send_sync(0x76, data, datalen);

    datalen = 0;
    data[datalen++] = 0x03;
    dc_spi_send_sync(0x77, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x78, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x79, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x7A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x7B, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0x80, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x81, data, datalen);

    datalen = 0;
    data[datalen++] = 0x05;
    dc_spi_send_sync(0x82, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x83, data, datalen);

    datalen = 0;
    data[datalen++] = 0xDD;
    dc_spi_send_sync(0x84, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x85, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x86, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x87, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0x88, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x89, data, datalen);

    datalen = 0;
    data[datalen++] = 0x07;
    dc_spi_send_sync(0x8A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x8B, data, datalen);

    datalen = 0;
    data[datalen++] = 0xDF;
    dc_spi_send_sync(0x8C, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x8D, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x8E, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x8F, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0x90, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x91, data, datalen);

    datalen = 0;
    data[datalen++] = 0x09;
    dc_spi_send_sync(0x92, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x93, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE1;
    dc_spi_send_sync(0x94, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x95, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x96, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x97, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0x98, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x99, data, datalen);

    datalen = 0;
    data[datalen++] = 0x0B;
    dc_spi_send_sync(0x9A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0x9B, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE3;
    dc_spi_send_sync(0x9C, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x9D, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x9E, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x9F, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0xA0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xA1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x04;
    dc_spi_send_sync(0xA2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xDC;
    dc_spi_send_sync(0xA4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xA5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xA6, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xA7, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0xA8, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xA9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x06;
    dc_spi_send_sync(0xAA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0xAB, data, datalen);

    datalen = 0;
    data[datalen++] = 0xDE;
    dc_spi_send_sync(0xAC, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xAD, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xAE, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xAF, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0xB0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xB1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x08;
    dc_spi_send_sync(0xB2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0xB3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE0;
    dc_spi_send_sync(0xB4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xB5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xB6, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xB7, data, datalen);

    datalen = 0;
    data[datalen++] = 0x48;
    dc_spi_send_sync(0xB8, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xB9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x0A;
    dc_spi_send_sync(0xBA, data, datalen);

    datalen = 0;
    data[datalen++] = 0x02;
    dc_spi_send_sync(0xBB, data, datalen);

    datalen = 0;
    data[datalen++] = 0xE2;
    dc_spi_send_sync(0xBC, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xBD, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xBE, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xBF, data, datalen);

    datalen = 0;
    data[datalen++] = 0x12;
    dc_spi_send_sync(0xC0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x88;
    dc_spi_send_sync(0xC1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x65;
    dc_spi_send_sync(0xC2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x74;
    dc_spi_send_sync(0xC3, data, datalen);

    datalen = 0;
    data[datalen++] = 0x47;
    dc_spi_send_sync(0xC4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x56;
    dc_spi_send_sync(0xC5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xC6, data, datalen);

    datalen = 0;
    data[datalen++] = 0xAA;
    dc_spi_send_sync(0xC7, data, datalen);

    datalen = 0;
    data[datalen++] = 0xBB;
    dc_spi_send_sync(0xC8, data, datalen);

    datalen = 0;
    data[datalen++] = 0x33;
    dc_spi_send_sync(0xC9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x21;
    dc_spi_send_sync(0xD0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x88;
    dc_spi_send_sync(0xD1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x65;
    dc_spi_send_sync(0xD2, data, datalen);

    datalen = 0;
    data[datalen++] = 0x74;
    dc_spi_send_sync(0xD3, data, datalen);

    datalen = 0;
    data[datalen++] = 0x47;
    dc_spi_send_sync(0xD4, data, datalen);

    datalen = 0;
    data[datalen++] = 0x56;
    dc_spi_send_sync(0xD5, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xD6, data, datalen);

    datalen = 0;
    data[datalen++] = 0xAA;
    dc_spi_send_sync(0xD7, data, datalen);

    datalen = 0;
    data[datalen++] = 0xBB;
    dc_spi_send_sync(0xD8, data, datalen);

    datalen = 0;
    data[datalen++] = 0x33;
    dc_spi_send_sync(0xD9, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0xF3, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0xF1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x0B;
    dc_spi_send_sync(0xA0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x2A;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x2B;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x2C;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x2D;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x2E;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x2F;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x30;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x31;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x32;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x33;
    dc_spi_send_sync(0xA3, data, datalen);

    datalen = 0;
    data[datalen++] = 0xC3;
    dc_spi_send_sync(0xA5, data, datalen);
    delay_ms(1);

    datalen = 0;
    data[datalen++] = 0x09;
    dc_spi_send_sync(0xA0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x10;
    dc_spi_send_sync(0xF1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0xF0, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x35, data, datalen);

    dc_spi_send_sync(0xF4, NULL, 0);

    datalen = 0;
    data[datalen++] = 0x20;
    dc_spi_send_sync(0xC1, data, datalen);

    datalen = 0;
    data[datalen++] = 0x60;
    dc_spi_send_sync(0xC2, data, datalen);

    dc_spi_send_sync(0xF4, NULL, 0);

    datalen = 0;
    data[datalen++] = 0x01;
    data[datalen++] = 0x7c;
    dc_spi_send_sync(0x44, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x36, data, datalen);

    datalen = 0;
    data[datalen++] = 0x55;
    dc_spi_send_sync(0x3A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    data[datalen++] = 0x00;
    data[datalen++] = 0x01;
    data[datalen++] = 0x67;
    dc_spi_send_sync(0x2A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    data[datalen++] = 0x00;
    data[datalen++] = 0x01;
    data[datalen++] = 0x67;
    dc_spi_send_sync(0x2B, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x4D, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x4E, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x4F, data, datalen);

    datalen = 0;
    data[datalen++] = 0x01;
    dc_spi_send_sync(0x4C, data, datalen);
    delay_ms(10);

    datalen = 0;
    data[datalen++] = 0x00;
    dc_spi_send_sync(0x4C, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    data[datalen++] = 0x00;
    data[datalen++] = 0x01;
    data[datalen++] = 0x67;
    dc_spi_send_sync(0x2A, data, datalen);

    datalen = 0;
    data[datalen++] = 0x00;
    data[datalen++] = 0x00;
    data[datalen++] = 0x01;
    data[datalen++] = 0x67;
    dc_spi_send_sync(0x2B, data, datalen);

    dc_spi_send_sync(0x21, NULL, 0);
    dc_spi_send_sync(0x11, NULL, 0);
    delay_ms(120);

    dc_spi_send_sync(0x29, NULL, 0);
}

static void display_te_set_enable(bool enabled)
{
    if (enabled)
    {
#if DISPLAY_TE_IO_TYPE == APP_IO_TYPE_AON
        ll_aon_gpio_enable_it(DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOA
        ll_gpio_enable_it(GPIO0, DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOB
        ll_gpio_enable_it(GPIO1, DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOC
        ll_gpio_enable_it(GPIO2, DISPLAY_TE_IO_PIN);
#endif // DISPLAY_TE_IO_TYPE == APP_IO_TYPE_AON
    }
    else
    {
#if DISPLAY_TE_IO_TYPE == APP_IO_TYPE_AON
        ll_aon_gpio_disable_it(DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOA
        ll_gpio_disable_it(GPIO0, DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOB
        ll_gpio_disable_it(GPIO1, DISPLAY_TE_IO_PIN);
#elif DISPLAY_TE_IO_TYPE == APP_IO_TYPE_GPIOC
        ll_gpio_disable_it(GPIO2, DISPLAY_TE_IO_PIN);
#endif // DISPLAY_TE_IO_TYPE == APP_IO_TYPE_AON
    }
}

static void display_te_evt_callback(app_io_evt_t *p_evt)
{
#ifdef USE_OSAL
    osal_sema_give(s_te_sem);
#else // USE_OSAL
    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(s_te_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif // USE_OSAL
}
