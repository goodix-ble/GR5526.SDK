#ifndef _TP_CONFIG_H_
#define _TP_CONFIG_H_

#include "app_io.h"
#include <stdbool.h>
#include "board_SK.h"

#define TP_I2C_ADDR          (0x2C)

typedef void (*tp_int_cb_func_t)(void);

bool tp_write(uint8_t reg_addr, uint8_t value);
bool tp_read(uint8_t reg_addr, uint8_t *buffer, uint16_t len);
void tp_config_init(tp_int_cb_func_t tp_int_cb);

#endif
