#ifndef __TP_CST816D_DRIVER_H__
#define __TP_CST816D_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

typedef void (*tp_int_evt_cb_t)(void);

/**
 * @brief Initialize the CST816D touch panel.
 */
void tp_cst816d_init(tp_int_evt_cb_t cb);

/**
 * @brief Deinitialize the CST816D touch panel.
 */
void tp_cst816d_deinit(void);

/**
 * @brief Read the touch pointer coordinates from the CST816D touch panel.
 *
 * @param px Pointer to store the X coordinate.
 * @param py Pointer to store the Y coordinate.
 * @return True if successful, false otherwise.
 */
bool tp_cst816d_read_pointer(int16_t *px, int16_t *py);

/**
 * @brief Put the CST816D touch panel into sleep mode.
 *
 * @return True if successful, false otherwise.
 */
bool tp_cst816d_sleep(void);

/**
 * @brief Wake up the CST816D touch panel from sleep mode.
 *
 * @return True if successful, false otherwise.
 */
bool tp_cst816d_wakeup(void);

#endif // __TP_CST816D_DRIVER_H__
