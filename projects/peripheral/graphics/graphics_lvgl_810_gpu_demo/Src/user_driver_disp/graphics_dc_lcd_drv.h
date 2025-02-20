/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup GRAPHICS_DC_LCD_DRV LCD Driver reference design for DC Module.
  * @{
  */

#ifndef __GRAPHICS_DC_LCD_DRV_H__
#define __GRAPHICS_DC_LCD_DRV_H__

#include "grx_sys.h"
#include "app_graphics_dc.h"

/**
  * @addtogroup GRAPHICS_DC_LCD_DRV_MACRO Defines
  * @{
  */
/** @defgroup GRAPHICS_DC_LCD_DRV_RES_DEFINE The LCD resolution definition
  * @{
  */
#define LCD_RES_390         0         /**< The resolution is 390X390 */
#define LCD_RES_454         1         /**< The resolution is 454X454*/
#define LCD_RES_360         2         /**< The resolution is 360X360*/
/** @} */

/** @defgroup GRAPHICS_DC_IS_SK_BOARD_DEFINE The board definition
  * @{
  */
#define IS_SK_BOARD         1       /* 0 - EVB Board
                                     * 1 - SK Board
                                     */
/** @} */

/** @defgroup GRAPHICS_DC_LCD_DRV_RES_TYPE Typedef
  * @{
  */
typedef uint32_t lcd_res_e;     /**< The LCD resolution type  */
/** @} */

/** @} */

/** @addtogroup GRAPHICS_DC_LCD_DRV_ENUM Enumerations
  * @{
  */
/**
  * @brief Lcd pixel mode
  */
typedef enum {
    LCD_PIXEL_mode_16bit = 0,        /**< The lcd pixel is 16bits */
    LCD_PIXEL_mode_24bit,            /**< The lcd pixel is 24bits */
} lcd_pixel_mode_e;
/** @} */

/**
 * @defgroup GRAPHICS_DC_LCD_DRV_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief DC rm69330 qspi lcd init
 *
 * @param[in] res: The lcd resolution
 * @param[in] pixel_mode: The lcd pixel mode
 * @param[in] mipi_format: The DC output mode
 *****************************************************************************************
 */
void graphics_dc_rm69330_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format) ;

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd test
 *
 * @param[in] res: The lcd resolution
 * @param[in] pixel_mode: The lcd pixel mode
 * @param[in] frame_timing: The DC output frame timing
 * @param[in] mipi_format: The DC output mode
 *****************************************************************************************
 */
void graphics_dc_rm69330_qspi_lcd_test(lcd_res_e res, lcd_pixel_mode_e pixel_mode, app_graphics_dc_frame_timing_e frame_timing, graphics_dc_mipi_format_e mipi_format);

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd set show area for qspi output
 *
 * @param[in] x1: The X start
 * @param[in] x2: The X end
 * @param[in] y1: The Y start
 * @param[in] y2: The Y end
 *****************************************************************************************
 */
void graphics_dc_rm69330_qspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd send one frame data
 *
 * @param[in] layer0: The Layer number
 * @param[in] lcd_w: The W resolution
 * @param[in] lcd_h: The H resolution
 *****************************************************************************************
 */
void graphics_dc_rm69330_qspi_send_frame(app_graphics_dc_framelayer_t layer0, uint32_t lcd_w, uint32_t lcd_h);

/**
 *****************************************************************************************
 * @brief DC rm69330 dspi lcd init
 *
 * @param[in] res: The lcd resolution
 * @param[in] pixel_mode: The lcd pixel mode
 * @param[in] mipi_format: The DC output mode
 *****************************************************************************************
 */
void graphics_dc_st7789_dspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format);

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd set show area for dspi output
 *
 * @param[in] x1: The X start
 * @param[in] x2: The X end
 * @param[in] y1: The Y start
 * @param[in] y2: The Y end
 *****************************************************************************************
 */
void graphics_dc_st7789_dspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/**
 *****************************************************************************************
 * @brief DC rm69330 spi lcd init
 *
 * @param[in] res: The lcd resolution
 * @param[in] pixel_mode: The lcd pixel mode
 * @param[in] mipi_format: The DC output mode
 *****************************************************************************************
 */
void graphics_dc_rm69330_spi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format);

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd set show area for spi output
 *
 * @param[in] x1: The X start
 * @param[in] x2: The X end
 * @param[in] y1: The Y start
 * @param[in] y2: The Y end
 *****************************************************************************************
 */
void graphics_dc_rm69330_spi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/**
 *****************************************************************************************
 * @brief DC rm69330 lcd set screen off
 *****************************************************************************************
 */
void graphics_dc_rm69330_screen_off(void);
/**
 *****************************************************************************************
 * @brief DC rm69330 lcd set screen on
 *****************************************************************************************
 */
void graphics_dc_rm69330_screen_on(void);



/**
 *****************************************************************************************
 * @brief DC fls-amo139 qspi lcd init
 *
 * @param[in] res: The lcd resolution
 * @param[in] pixel_mode: The lcd pixel mode
 * @param[in] mipi_format: The DC output mode
 *****************************************************************************************
 */
void graphics_dc_am139_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format);


/**
 *****************************************************************************************
 * @brief DC fls-amo139 lcd set show area for qspi output
 *
 * @param[in] x1: The X start
 * @param[in] x2: The X end
 * @param[in] y1: The Y start
 * @param[in] y2: The Y end
 *****************************************************************************************
 */
void graphics_dc_am139_qspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/**
 *****************************************************************************************
 * @brief DC fls-amo139 lcd send one frame data
 *
 * @param[in] layer0: The Layer number
 * @param[in] lcd_w: The W resolution
 * @param[in] lcd_h: The H resolution
 *****************************************************************************************
 */
void graphics_dc_am139_qspi_send_frame(app_graphics_dc_framelayer_t layer0, uint32_t lcd_w, uint32_t lcd_h);




/** @} */

#endif /* __GRAPHICS_DC_LCD_DRV_H__ */

/** @} */
/** @} */

