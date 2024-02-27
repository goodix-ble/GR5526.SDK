#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "graphics_defs.h"
#include "graphics_dc_lcd_drv.h"
#include "lv_port_disp.h"
#include "app_qspi.h"
#include "qspi_flash.h"

/*
 * Flash Device Setting
 */
#define Q_NOR_FLASH_QSPI_ID                         APP_QSPI_ID_0
#define Q_NOR_FLASH_CLOCK_PREESCALER                2u
#define Q_NOR_FLASH_PIN_GROUP                       QSPI0_PIN_GROUP_0

/*
 * FPS Configuration
 */
#define LV_SHOW_FPS_IN_LABEL                        1
#define LV_SHOW_FPS_IN_UART                         0
#define LV_CALC_AVG_FPS                             1
#define LV_FPS_CACHE_NB                             4

/*
 * EXTERNAL DECLARATION
 */
extern void lv_port_set_fb_format(uint32_t format);
extern uint32_t lv_port_get_fb_format(void);
extern uint32_t sys_cpu_usage(void);

//typedef struct {
static TLS_VAR app_graphics_dc_framelayer_t s_dc_layer =
{
    .frame_baseaddr   = NULL,
    .resolution_x     = DISP_HOR_RES,
    .resolution_y     = DISP_HOR_RES,
    .row_stride       = DISP_HOR_RES * 2,
    .start_x          = 0,
    .start_y          = 0,
    .size_x           = DISP_HOR_RES,
    .size_y           = DISP_VER_RES,
    .alpha            = 0,
    .blendmode        = HAL_GDC_BL_SRC,
    .data_format      = GDC_DATA_FORMAT_RGB565
};

static uint8_t s_fps_save[LV_FPS_CACHE_NB];
static uint32_t s_refresh_fps;
static bool s_debug_info_enable = false;

void lv_port_debug_info_enable(bool enable){
    s_debug_info_enable = enable;
}

static void lv_fps_update(void){
    static uint32_t s_last_flush_tick_cnt = 0;
    static uint32_t s_cur_flush_tick_cnt = 0;
    s_cur_flush_tick_cnt = lv_tick_get();
    uint32_t render_diff_ms = s_cur_flush_tick_cnt - s_last_flush_tick_cnt;
    s_last_flush_tick_cnt = s_cur_flush_tick_cnt;
    s_refresh_fps = 1000 / render_diff_ms;

#if LV_CALC_AVG_FPS
    for(uint8_t i=0; i < LV_FPS_CACHE_NB-1; i++){
        s_fps_save[i] = s_fps_save[i+1];
    }
    s_fps_save[LV_FPS_CACHE_NB-1] = s_refresh_fps;
    uint32_t fps_sum = 0;
    for(uint8_t i = 0; i < LV_FPS_CACHE_NB; i++){
        fps_sum += s_fps_save[i];
    }
    #if (4 == LV_FPS_CACHE_NB)
    s_refresh_fps = fps_sum >> 2;
    #else
    s_refresh_fps = fps_sum / LV_FPS_CACHE_NB;
    #endif
#endif
}

void lv_dc_mode_update(void){
    if(s_refresh_fps > 25){ // update the frequency to 48Mhz for peformance
        app_graphics_dc_freq_set(GDC_CLOCK_FREQ_48MHz);
    }else{ // update the frequency to 24Mhz to ease the bus stress
        app_graphics_dc_freq_set(GDC_CLOCK_FREQ_24MHz);
    }
}

void lv_refresh_fps_draw(void){
    if(!s_debug_info_enable) return;
    char info[20] = {0};
    sprintf(info,"FPS: %d, CPU: %d", s_refresh_fps, sys_cpu_usage());
#if LV_SHOW_FPS_IN_LABEL
    lv_draw_label_dsc_t draw_label_dsc;
    lv_draw_label_dsc_init(&draw_label_dsc);
    draw_label_dsc.color = lv_color_white();
    draw_label_dsc.font = &lv_font_montserrat_20;
    lv_area_t coords = {.x1 = 160, .y1=100, .x2=400, .y2=200};
    lv_draw_label(&coords, &coords, &draw_label_dsc, info, NULL);
#elif LV_SHOW_FPS_IN_UART
    puts(info);
#endif
}

void lv_transition_fps_draw(void){
    if(!s_debug_info_enable) return;
    uint32_t format = lv_port_get_fb_format();
    lv_port_set_fb_format(HAL_GFX_TSC4);
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t* cl = &cmd;
    hal_gfx_cl_bind_circular(cl);
    lv_refresh_fps_draw();
    hal_gfx_cl_le_destroy(cl);
    lv_port_set_fb_format(format);
}

void gx_dc_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p){
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    lv_fps_update();
    lv_dc_mode_update();
    lv_refresh_fps_draw();
    app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    s_dc_layer.frame_baseaddr = draw_buf->buf_act;
    app_graphics_dc_cmd_t   dc_cmd;
    dc_cmd.command       = 0x12;
    dc_cmd.address       = 0x002C00;
    dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT; /*SYNC with HAL_GDC_RGB565*/
    s_dc_layer.data_format    = (graphics_dc_data_format_e)HAL_GDC_RGB565;
    dc_cmd.frame_timing  = GDC_QSPI_FRAME_TIMING_1;
    s_dc_layer.resolution_y   = area->y2 - area->y1;
    s_dc_layer.size_y         = s_dc_layer.resolution_y;
#if LCD_IS_FLS_AMO139 > 0u
    graphics_dc_am139_qspi_lcd_set_show_area(area->x1, area->x2 - area->x1, area->y1, area->y2 - area->y1);
#else
    graphics_dc_rm69330_qspi_lcd_set_show_area(area->x1, area->x2 - area->x1, area->y1, area->y2 - area->y1);
#endif
    app_graphics_dc_send_single_frame(GRAPHICS_DC_LAYER_0, &s_dc_layer, &dc_cmd, GDC_ACCESS_TYPE_ASYNC);
}

void gx_dc_flush_transition(void* addr, uint32_t w, uint32_t h, uint32_t format){
    app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    lv_fps_update();
    app_graphics_dc_freq_set(GDC_CLOCK_FREQ_48MHz);
    lv_transition_fps_draw();
    app_graphics_dc_cmd_t   dc_cmd;
    dc_cmd.command       = 0x12;
    dc_cmd.address       = 0x002C00;
    dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT; /*SYNC with HAL_GDC_RGB565*/
    dc_cmd.frame_timing  = GDC_QSPI_FRAME_TIMING_1;
    app_graphics_dc_framelayer_t dc_layer = {
        .frame_baseaddr   = addr,
        .resolution_x     = w,
        .resolution_y     = h,
        .row_stride       = -1,
        .start_x          = 0,
        .start_y          = 0,
        .size_x           = w,
        .size_y           = h,
        .alpha            = 0,
        .blendmode        = HAL_GDC_BL_SRC,
        .data_format      = (graphics_dc_data_format_e)format,
    };
#if LCD_IS_FLS_AMO139 > 0u
    graphics_dc_am139_qspi_lcd_set_show_area(0, w - 1, 0, h - 1);
#else
    graphics_dc_rm69330_qspi_lcd_set_show_area(0, w - 1, 0, h - 1);
#endif
    app_graphics_dc_send_single_frame(GRAPHICS_DC_LAYER_0, &dc_layer, &dc_cmd, GDC_ACCESS_TYPE_ASYNC);
}


void gx_dc_init(){
    graphics_gpu_init(NULL);
#if LCD_IS_FLS_AMO139 > 0u
    graphics_dc_am139_qspi_lcd_init(LCD_RES_CURRENT, LCD_PIXEL_mode_16bit, GDC_MIPICFG_QSPI_RGB565_OPT0);
#else
    graphics_dc_rm69330_qspi_lcd_init(LCD_RES_CURRENT, LCD_PIXEL_mode_16bit, GDC_MIPICFG_QSPI_RGB565_OPT0);
#endif
    uint8_t flash_id = SPI_FLASH_init(Q_NOR_FLASH_QSPI_ID, Q_NOR_FLASH_CLOCK_PREESCALER, Q_NOR_FLASH_PIN_GROUP);
    if(flash_id != 0x0B) {
        printf("Flash init error\r\n");
    }
    app_qspi_mmap_device_t dev = {
        .dev_type    = APP_QSPI_DEVICE_FLASH,
        .rd.flash_rd = FLASH_MMAP_CMD_4READ_EBH,
    };
    SPI_FLASH_Enable_Quad();
    app_qspi_config_memory_mappped(Q_NOR_FLASH_QSPI_ID, dev);
    app_qspi_mmap_set_endian_mode(Q_NOR_FLASH_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_1);
}

void lv_port_res_mode_set(uint8_t mode){
    if(mode == 0){
        app_qspi_mmap_set_endian_mode(Q_NOR_FLASH_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_0);
    }else if(mode == 1){
        app_qspi_mmap_set_endian_mode(Q_NOR_FLASH_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_1);
    }else if(mode == 2){
        app_qspi_mmap_set_endian_mode(Q_NOR_FLASH_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_2);
    }
}
