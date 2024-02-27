#include "app_gpiote.h"
#include "app_io.h"
#include "dspi_screen_390.h"
#include "board_SK.h"

static void dspi_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

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
        .baud_rate = DSPI_BAUD_RATE_128P1PCLK,
        .dspi_mode = DSPI_PROT_MODE_4W1L,
    },
    .is_soft_cs = true,
};
/* misc pins */

#define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_OUTPUT,  APP_IO_NOPULL, NULL}
#define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
#define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_7, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}

/*
 * LOCAL VARIABLE DEFINITIONS
 ***************************************************************
 *
 */
static volatile uint8_t                     g_master_tdone = 0;
static dspi_command_t                       g_cmd;
volatile uint8_t                            display_busy = 0;
static volatile uint32_t                    display_xfer_remain = 0;
static uint8_t                              *display_xfer_point = NULL;
#define DSPI_DMA_SIZE                       (4094 * 2)

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

static void app_dspi_display_4w2l_write(uint8_t *data, uint32_t dataLen)
{
    // Write_Command(0x2C);
    app_dspi_config_mode(DSPI_PROT_MODE_4W1L);
    g_cmd.instruction = 0x2C;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 0;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);

    display_xfer_remain = dataLen;
    display_xfer_point = data;
    display_busy = 1;

    app_dspi_config_mode(DSPI_PROT_MODE_4W2L);
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

static void app_dspi_display_4w1l_write(uint8_t *data, uint32_t dataLen)
{
    // Write_Command(0x2C);
    g_cmd.instruction = 0x2C;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 0;

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

static void app_dspi_display_3w1l_write(uint8_t *data, uint32_t dataLen)
{
    // Write_Command(0x2C);
    g_cmd.instruction = 0x2C;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_16_BITS;
    g_cmd.length = 0;

    g_master_tdone = 0;
    app_dspi_dma_command_async(&g_cmd);
    while(g_master_tdone == 0);

    display_xfer_remain = dataLen;
    display_xfer_point = data;
    display_busy = 1;

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

static uint32_t screen_init(app_dspi_evt_handler_t evt_handler)
{
    uint16_t ret = APP_DRV_ERR_HAL;

    ret = app_dspi_init(&dspi_params, evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Initialize DSPI failed\r\n");
    }
    delay_ms(200);

    ret = app_dspi_dma_init(&dspi_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Initialize DSPI DMA failed\r\n");
    }
    delay_ms(200);

    return ret;
}

static void lcd_io_init(app_gpiote_param_t io_para)
{
    app_io_init_t io_init;
    io_init.pin  = io_para.pin;
    io_init.mode = io_para.mode;
    io_init.pull = io_para.pull;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
    io_init.mux  = APP_IO_MUX_7;
#else
    io_init.mux  = APP_IO_MUX_8;
#endif
    app_io_init(io_para.type, &io_init);
}

static void qspi_screen_misc_pins_init(void)
{
    app_gpiote_param_t io_para1 = DISP_RST_CONFIG;
    lcd_io_init(io_para1);
    app_gpiote_param_t io_para2 = DISP_TE_CONFIG;
    lcd_io_init(io_para2);
    app_gpiote_param_t io_para3 = DISP_DC_CONFIG;
    lcd_io_init(io_para3);
}

static void lcd_rst_ctrl(uint8_t level)
{
    app_io_pin_state_t pin_state;
    app_gpiote_param_t rst_pin = DISP_RST_CONFIG;
    app_io_type_t pin_type = rst_pin.type;
    uint32_t pin = rst_pin.pin;

    pin_state = level ? APP_IO_PIN_SET : APP_IO_PIN_RESET;
    app_io_write_pin(pin_type, pin, pin_state);
}

static void lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len)
{
    g_cmd.instruction = cmd;
    g_cmd.instruction_size = DSPI_INSTSIZE_08_BITS;
    g_cmd.data_size = DSPI_DATASIZE_08_BITS;
    g_cmd.length = data_len;

    g_master_tdone = 0;
    app_dspi_dma_command_transmit_async(&g_cmd, data);
    while(g_master_tdone == 0);
}

static void lcd_cmd_send(uint8_t cmd, uint8_t param)
{
    lcd_write(cmd, &param, 1);
}

static void lcd_cmd_sequence(uint8_t mode)
{
    switch(mode) {
        case D_DISPLAT_4W1L:
        {
            lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            lcd_cmd_send(0xc4, 0x80);  //Set_DSPI Mode
            lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            lcd_cmd_send(0x51, 0xff);  //display brightness
            dspi_screen_set_show_area(0, 454 - 1, 0, 454 - 1);
            lcd_cmd_send(0x3a, 0x55);  //0x55:RGB565  0x66:RGB666  0x77:RGB888 

            lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(120);
            //lcd_cmd_send(0x23, 0x00);  //all pixel on
            lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(80);
            //lcd_cmd_send(0x12);  //Partial display mode on
        }
        break;

        case D_DISPLAT_4W2L:
        {
            lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            lcd_cmd_send(0xc4, 0xa1);  //Set_DSPI Mode
            lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            lcd_cmd_send(0x51, 0xff);  //display brightness
            dspi_screen_set_show_area(0, 454 - 1, 0, 454 - 1);
            lcd_cmd_send(0x3a, 0x55);  //0x55:RGB565  0x66:RGB666  0x77:RGB888 

            lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(120);
            //lcd_cmd_send(0x23, 0x00);  //all pixel on
            lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(80);
            //lcd_cmd_send(0x12);  //Partial display mode on
        }
        break;

        case D_DISPLAT_3W1L:
        {
            app_dspi_config_mode(LL_DSPI_PROT_MODE_3W1L);
            delay_ms(80);
            lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            lcd_cmd_send(0xc4, 0x81);  //Set_DSPI Mode
            lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            lcd_cmd_send(0x51, 0xff);  //display brightness
            dspi_screen_set_show_area(0, 454 - 1, 0, 454 - 1);
            lcd_cmd_send(0x3a, 0x55);  //0x55:RGB565  0x66:RGB666  0x77:RGB888 

            lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(120);
            //lcd_cmd_send(0x23, 0x00);  //all pixel on
            lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(80);
            //lcd_cmd_send(0x12);  //Partial display mode on
            app_dspi_config_data_size(DSPI_DATASIZE_16_BITS);
        }
        break;

        default:
        {}
        break;
    }
}

static void lcd_ca_ra_set(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint8_t data[4];

    data[0] = (x1 & 0xff00) >> 8;
    data[1] = x1 & 0x00ff;
    data[2] = (x2 & 0xff00) >> 8;
    data[3] = x2 & 0x00ff;
    lcd_write(0x2a, data, 4);

    data[0] = (y1 & 0xff00) >> 8;
    data[1] = y1 & 0x00ff;
    data[2] = (y2 & 0xff00) >> 8;
    data[3] = y2 & 0x00ff;
    lcd_write(0x2b, data, 4);
}

static void dspi_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    lcd_ca_ra_set(x1, x2, y1, y2);
}

/*
 * GLOBAL FUNCTION
 ***************************************************************
 *
 */
void display_lcd(uint8_t *data, uint32_t dataLen, uint8_t mode)
{
    if (mode == D_DISPLAT_4W1L)
    {
        app_dspi_display_4w1l_write(data, dataLen);
    } 
    else if (mode == D_DISPLAT_4W2L)
    {
        app_dspi_display_4w2l_write(data, dataLen);
    } 
    else if (mode == D_DISPLAT_3W1L)
    {
        app_dspi_display_3w1l_write(data, dataLen);
    }

    while(display_busy == 1);
}

void dspi_screen_init_basic(uint8_t mode)
{
    screen_init(app_dspi_evt_handler);

    qspi_screen_misc_pins_init();
    lcd_rst_ctrl(0);
    delay_ms(100);
    lcd_rst_ctrl(1);
    delay_ms(100);

    lcd_cmd_sequence(mode);
}

void dspi_screen_deinit(void)
{
    uint16_t ret = APP_DRV_ERR_HAL;

    ret = app_dspi_dma_deinit();
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Deinitialize DSPI DMA failed\r\n");
    }
    
    ret = app_dspi_deinit();
    if (ret != APP_DRV_SUCCESS)
    {
        printf("Deinitialize DSPI failed\r\n");
    }
}
