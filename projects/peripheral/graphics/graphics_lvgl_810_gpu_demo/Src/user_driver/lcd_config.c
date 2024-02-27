#include "lcd_config.h"
#include "app_qspi.h"
#include "app_gpiote.h"
#include "grx_hal.h"

static disp_tx_cb_t s_disp_tx_cb;
static volatile bool is_need_notify = false;

static void lcd_qspi_callback(app_qspi_evt_t *p_evt)
{
    switch(p_evt->type)
    {
#if 0
        case APP_QSPI_EVT_TX_CPLT:
            if ( s_disp_tx_cb.in_use == false)
            {
                return;
            }

            s_disp_tx_cb.p_data += s_disp_tx_cb.len;
            s_disp_tx_cb.remain -= s_disp_tx_cb.len;
            s_disp_tx_cb.len = (s_disp_tx_cb.remain >= DMA_MAX_LEN) ? DMA_MAX_LEN : s_disp_tx_cb.remain;
            if (s_disp_tx_cb.remain)
            {
                lcd_data_transmit(0x003c00, s_disp_tx_cb.p_data, s_disp_tx_cb.len);
            }
            else
            {
                if (s_disp_tx_cb.tx_cplt_cb != NULL)
                {
                    s_disp_tx_cb.tx_cplt_cb(s_disp_tx_cb.tx_cplt_cb_param);
                }
                s_disp_tx_cb.in_use = false;
                assistant_switch_pin_state(APP_IO_PIN_4, 0);
            }
            break;
#endif
        case APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT:
        {
            printf("V");
//            lv_enhanced_sem_post();
            if (s_disp_tx_cb.tx_cplt_cb != NULL)
            {
                s_disp_tx_cb.tx_cplt_cb(s_disp_tx_cb.tx_cplt_cb_param);
            }
            is_need_notify = false;

        }
        break;

        case APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL:
        {
            printf(" XxX ");
//            lv_enhanced_sem_post();
            if (s_disp_tx_cb.tx_cplt_cb != NULL)
            {
                s_disp_tx_cb.tx_cplt_cb(s_disp_tx_cb.tx_cplt_cb_param);
            }
            is_need_notify = false;
        }
        break;

        default:break;
    }
}

void lcd_disp_complete(void) {
    if (s_disp_tx_cb.tx_cplt_cb != NULL)
    {
        s_disp_tx_cb.tx_cplt_cb(s_disp_tx_cb.tx_cplt_cb_param);
    }
    is_need_notify = false;
}

void lcd_config_init(void)
{
    app_qspi_params_t qspi_param = DISP_PARAM_CONFIG;
    app_qspi_init(&qspi_param, lcd_qspi_callback);
}


void lcd_rst_ctrl(uint8_t level)
{
    app_io_init_t io_init ;
    io_init.mode = APP_IO_MODE_OUT_PUT ;
    io_init.mux  = APP_IO_MUX_7;
    io_init.pin  = LCD_RESET_PIN;
    io_init.pull = APP_IO_PULLUP;
    
    app_io_init(LCD_RESET_GPIO, &io_init);
    
    if(level) {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_SET);
    } else {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_RESET);
    }
}


void lcd_write(uint8_t cmd, uint8_t *p_data, uint32_t data_len)
{
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction = 0x02;
    qspi_cmd.address = ((uint16_t)cmd) << 8;
    qspi_cmd.instruction_size = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles = 0;
    qspi_cmd.data_size = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    qspi_cmd.data_mode = QSPI_DATA_MODE_SPI;
    qspi_cmd.length = data_len;
    qspi_cmd.clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE;

    app_qspi_command_transmit(DISP_QSPI, &qspi_cmd, p_data, APP_QSPI_TYPE_DMA);
}

void lcd_write_u16(uint16_t cmd, uint8_t *p_data, uint32_t data_len)
{
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction = 0x02;
    qspi_cmd.address = cmd;
    qspi_cmd.instruction_size = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles = 0;
    qspi_cmd.data_size = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    qspi_cmd.data_mode = QSPI_DATA_MODE_SPI;
    qspi_cmd.length = data_len;
    qspi_cmd.clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE;

    app_qspi_command_transmit(DISP_QSPI, &qspi_cmd, p_data, APP_QSPI_TYPE_DMA);
}

void lcd_cmd_send(uint8_t cmd, uint8_t param)
{
    lcd_write(cmd, &param, 1);
}

void lcd_cmd_send_u16(uint16_t cmd, uint8_t param)
{
    lcd_write_u16(cmd, &param, 1);
}

void lcd_cmd_send_d(uint8_t cmd, uint8_t param, uint8_t param2)
{
    uint8_t p[2]={param,param2};
    lcd_write(cmd, p, 2);
}

void lcd_data_transmit(uint32_t addr, uint8_t *p_data, uint32_t len)
{
#if 1
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction = 0x12;
    qspi_cmd.address = addr;
    qspi_cmd.instruction_size = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles = 0;
    qspi_cmd.data_size = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF;
    qspi_cmd.data_mode = QSPI_DATA_MODE_QUADSPI;
    qspi_cmd.length = len;
    qspi_cmd.clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE;

    app_qspi_command_transmit(DISP_QSPI, &qspi_cmd, p_data, APP_QSPI_TYPE_DMA_ASYNC);
#else
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction = 0x02;
    qspi_cmd.address = addr;
    qspi_cmd.instruction_size = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles = 0;
    qspi_cmd.data_size = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    qspi_cmd.data_mode = QSPI_DATA_MODE_SPI;
    qspi_cmd.length = len;
    qspi_cmd.clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE;

    app_qspi_command_transmit(DISP_QSPI, &qspi_cmd, p_data, APP_QSPI_TYPE_DMA_ASYNC);
#endif
}


void lcd_init_disp_tx_cb(uint8_t *p_data, uint32_t len, void (*cb)(void *), void * cb_param)
{
    s_disp_tx_cb.p_data = p_data;
    s_disp_tx_cb.remain = len;
    s_disp_tx_cb.len = (len >= DMA_MAX_LEN) ? DMA_MAX_LEN : len;
    s_disp_tx_cb.tx_cplt_cb = cb;
    s_disp_tx_cb.tx_cplt_cb_param = cb_param;
    s_disp_tx_cb.in_use = true;

    is_need_notify = true;
}


void lcd_delay_ms(uint16_t ms)
{
    delay_ms(ms);
}

bool lcd_get_tx_cmp_flag(void)
{
    return s_disp_tx_cb.in_use;
}
