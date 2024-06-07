#ifndef __GRAPHICS_DC_ST77916_QSPI_DRV_H__
#define __GRAPHICS_DC_ST77916_QSPI_DRV_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the ST77916 using DC.
 *
 * @param screen_w Width of the screen.
 * @param screen_h Height of the screen.
 * @param pixel_format pixel format.
 */
void graphics_dc_st77916_init(uint16_t screen_w, uint16_t screen_h);

/**
 * @brief Deinitialize the ST77916 using DC.
 */
void graphics_dc_st77916_deinit(void);

/**
 * @brief Set the region to be refreshed on the screen.
 *
 * @param x1 Starting X coordinate of the display area.
 * @param x2 Ending X coordinate of the display area.
 * @param y1 Starting Y coordinate of the display area.
 * @param y2 Ending Y coordinate of the display area.
 */
void graphics_dc_st77916_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/**
 * @brief Flush the provided buffer to the ST77916 display.
 *
 * @param buf Pointer to the buffer containing the data to be displayed.
 * @param buf_format Format of the buffer data.
 * @param w Width of the display area in the buffer.
 * @param h Height of the display area in the buffer.
 */
void graphics_dc_st77916_flush(void *buf, uint32_t buf_format, uint16_t w, uint16_t h);

/**
 * @brief Turn on or off the ST77916.
 *
 * @param on Flag to control the display power state.
 */
void graphics_dc_st77916_set_on(bool on);

/**
 * @brief Wait for TE (Tearing Effect) signal.
 */
void graphics_dc_st77916_wait_te(void);

/**
 * @brief Wait for the ST77916 to be ready.
 */
void graphics_dc_st77916_wait_ready(void);

/**
 * @brief Put the ST77916 to sleep mode.
 */
void graphics_dc_st77916_sleep(void);

/**
 * @brief Wake up the ST77916 from sleep mode.
 */
void graphics_dc_st77916_wakeup(void);

/**
 * @brief Set screen brightness.
 *
 * @param percentage Brightness percentage.
 */
void graphics_dc_st77916_set_brightness(uint32_t percentage);

#endif // __GRAPHICS_DC_ST77916_QSPI_DRV_H__
