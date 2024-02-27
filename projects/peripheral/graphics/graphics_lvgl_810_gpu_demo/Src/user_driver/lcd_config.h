#ifndef _LCD_QSPI_CONFIG_H_
#define _LCD_QSPI_CONFIG_H_

#include "app_io.h"
#include <stdint.h>
#include <stdbool.h>


#define DISP_QSPI                        APP_QSPI_ID_2

#define DISP_MODE_CONFIG                 {APP_QSPI_TYPE_DMA, DMA1, DMA_Channel1, 3000, 0}
#define DISP_QSPI_CONFIG                 {2, QSPI_CLOCK_MODE_3, 0 }
#define DISP_PARAM_CONFIG                { DISP_QSPI, g_qspi_pin_groups[QSPI2_PIN_GROUP_0], DISP_MODE_CONFIG, DISP_QSPI_CONFIG }

#define LCD_RESET_GPIO                   APP_IO_TYPE_AON

/* misc pins */
#if 0 // EVB_BOARD
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_OUT_PUT, APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_1, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define LCD_RESET_PIN                     GPIO_PIN_5              /* Rest PIN in EVB Board */
    #define LCD_RESET_GPIO                    APP_IO_TYPE_AON
#else //SK Board
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_OUT_PUT, APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_7, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define LCD_RESET_PIN                      GPIO_PIN_6              /* Rest PIN in SK  Board */
    #define LCD_RESET_GPIO                     APP_IO_TYPE_AON
#endif
#define DISP_MISC_PINS_CONFIG            { DISP_RST_CONFIG, DISP_TE_CONFIG }


#define DMA_MAX_LEN                     (4092UL )
typedef struct 
{
    bool in_use;
    void (*tx_cplt_cb)(void *);
    void * tx_cplt_cb_param;
    uint8_t * p_data;
    uint32_t len;
    uint32_t remain;
} disp_tx_cb_t;


void lcd_config_init(void);
void lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len);
void lcd_cmd_send(uint8_t cmd, uint8_t param);
void lcd_cmd_send_u16(uint16_t cmd, uint8_t param);
void lcd_cmd_send_d(uint8_t cmd, uint8_t param, uint8_t param2);
void lcd_rst_ctrl(uint8_t level);
void lcd_data_transmit(uint32_t addr, uint8_t *p_data, uint32_t len);
void lcd_init_disp_tx_cb(uint8_t *p_data, uint32_t len, void (*cb)(void *), void * cb_param);
bool lcd_get_tx_cmp_flag(void);
void lcd_delay_ms(uint16_t ms);
#endif

