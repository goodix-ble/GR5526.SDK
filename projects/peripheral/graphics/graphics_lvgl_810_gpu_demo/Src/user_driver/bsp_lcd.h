#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__

#include <stdint.h>

void lcd_init(void);
void lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint8_t *p_data,void (*cb)(void *), void * cb_param);

#endif
