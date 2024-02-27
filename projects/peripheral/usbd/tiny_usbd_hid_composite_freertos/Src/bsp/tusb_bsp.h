#ifndef _TUSB_BSP_H_
#define _TUSB_BSP_H_

#ifdef __cplusplus
 extern "C" {
#endif
#include "stdint.h"
#include <stdbool.h>

void tusb_bsp_init(void);
uint32_t board_millis(void);
void board_led_write(bool state);
uint32_t board_button_read(void);
#ifdef __cplusplus
 }
#endif

#endif
