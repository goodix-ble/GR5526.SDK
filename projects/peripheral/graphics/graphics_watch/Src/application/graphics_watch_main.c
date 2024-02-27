#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"

#include "graphics_defs.h"
#include "graphics_dc_lcd_drv.h"

#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"

#if LCD_USE_HAL_GDC > 0
#include "hal_gdc.h"
#include "hal_gdc_mipi.h"
#endif

#include "hal_gfx_utils.h"
#include "hal_gfx_font.h"
#include "hal_gfx_graphics.h"
#include "string.h"
#include "app_graphics_mem.h"
#include "platform_sdk.h"

//#include "DejaVuSerif12pt8b.h"

#pragma diag_suppress   1296

#define SIMULATION_200    0
#define SIMULATION_360    1
#define SIMULATION_454    2

#define SIMULATION_WHICH    SIMULATION_454

#define IS_OSPI_PSRAM       1u      /* 0 - QSPI PSRAM; 1 - OSPI PSRAM */

#if SIMULATION_WHICH == SIMULATION_200
    #include "image/watch_panel_rgba_200x200p.h"
    #include "image/watch_needle_min_rgba_92x16p.h"

    #define RESX    200u
    #define RESY    200u

    #if RESX != 200
        #error "please set RESX/RESY to 200"
    #endif
#elif SIMULATION_WHICH == SIMULATION_360
    #include "image/watch_panel_rgba_360x360p.h"
    #include "image/watch_needle_min_rgba_200x20p.h"
    #include "image/needle_sec_01_200x12p_rgba.h"

    #define RESX    360u
    #define RESY    360u

    #if RESX != 360
        #error "please set RESX/RESY to 360"
    #endif
#elif SIMULATION_WHICH == SIMULATION_454
    #include "image/watch_panel_rgba_454x454p.h"
    #include "image/watch_needle_min_rgba_200x20p.h"
    #include "image/needle_sec_01_200x12p_rgba.h"

    #define RESX    454u
    #define RESY    454u

    #if RESX != 454
        #error "please set RESX/RESY to 454"
    #endif
#endif

#define IS_FB_TSCx          0u

static img_obj_t fb = {{0}, RESX, RESY, -1, 0, HAL_GFX_FB_FORMAT, 0};      /*RESX&RESX is 360, HAL_GFX_FB_FORMAT is RGBA8888 */

#if HAL_GDC_DISPLAY_ENABLE >0u
    static TLS_VAR app_graphics_dc_framelayer_t s_dc_layer = {(void *)0, RESX, RESY, -1, 0, 0, RESX, RESY, 0xff, HAL_GDC_BL_SRC, GDC_DATA_FORMAT_RGBA8888};
#endif

static img_obj_t s_watch_panel_img_cache;

const img_obj_t s_imgs_watch_panel = {
#if SIMULATION_WHICH == SIMULATION_200
    {
        .base_virt = (void *)       &watch_panel_rgba_200x200p_rgba[0],
        .base_phys = (uintptr_t)    &watch_panel_rgba_200x200p_rgba[0],
        .size      = (int)          watch_panel_rgba_200x200p_rgba_length
    },
    200, 200, -1, 0, HAL_GFX_RGBA8888, 0
#elif SIMULATION_WHICH == SIMULATION_360
    {
        .base_virt = (void *)       &demo_watch_panel_data[0],
        .base_phys = (uintptr_t)    &demo_watch_panel_data[0],
        .size      = (int)          demo_watch_panel_data_length
    },
    360, 360, -1, 0, HAL_GFX_RGBA8888, 0
#elif SIMULATION_WHICH == SIMULATION_454
    {
        .base_virt = (void *)       &watch_panel_rgba_454p_1[0],
        .base_phys = (uintptr_t)    &watch_panel_rgba_454p_1[0],
        .size      = (int)          watch_panel_rgba_454p_length
    },
    454, 454, -1, 0, HAL_GFX_RGBA8888, 0
#endif
};

const img_obj_t s_imgs_watch_min_needle = {
#if SIMULATION_WHICH == SIMULATION_200
    {
            .base_virt = (void *)       &watch_needle_min_rgba_92x16p_rgba[0],
            .base_phys = (uintptr_t)    &watch_needle_min_rgba_92x16p_rgba[0],
            .size      = (int)          watch_needle_min_rgba_92x16p_rgba_length
        },
        92, 16, -1, 0, HAL_GFX_RGBA8888, 0
#else
    #if 1
        {
            .base_virt = (void *)       &needle_sec_01_200x12p_rgba[0],
            .base_phys = (uintptr_t)    &needle_sec_01_200x12p_rgba[0],
            .size      = (int)          needle_sec_01_200x12p_rgba_length
        },
        200, 12, -1, 0, HAL_GFX_RGBA8888, 0
    #else
        {
            .base_virt = (void *)       &demo_watch_needle_min_rgba[0],
            .base_phys = (uintptr_t)    &demo_watch_needle_min_rgba[0],
            .size      = (int)          demo_watch_needle_min_rgba_length
        },
        200, 20, -1, 0, HAL_GFX_RGBA8888, 0
    #endif
#endif
};


static hal_gfx_buffer_t hal_gfx_fb_create (int size)
{
    hal_gfx_buffer_t bo;
    mem_pwr_mgmt_mode_set(MEM_POWER_FULL_MODE);

    bo.base_virt = app_graphics_mem_malloc(size);
    bo.base_phys = (uintptr_t) (bo.base_virt);
    bo.size      = size;
    bo.fd        = 0;

    return bo;
}

#if SIMULATION_WHICH == SIMULATION_454

#define IMAGE_BUFFER_ADDRESS 0x30280000

static hal_gfx_buffer_t hal_gfx_imagebuffer_create (int size)
{
    hal_gfx_buffer_t bo;

    bo.base_virt = (void*)IMAGE_BUFFER_ADDRESS;
    bo.base_phys = (uintptr_t) (bo.base_virt);
    bo.size      = size;
    bo.fd        = 0;

    return bo;
}
#endif

//static hal_gfx_buffer_t hal_gfx_fontbuffer_create (int size)
//{
//    hal_gfx_buffer_t bo;

//    bo.base_virt = (void*)FONT_BUFFER_ADDRESS;
//    bo.base_phys = (uintptr_t) (bo.base_virt);
//    bo.size      = size;
//    bo.fd        = 0;

//    return bo;
//}

#if 0
static void load_font_object(void) {
    DejaVuSerif12pt8b.bo = hal_gfx_fontbuffer_create(DejaVuSerif12pt8b.bitmap_size);
    hal_gfx_buffer_map(&DejaVuSerif12pt8b.bo);
    memcpy(DejaVuSerif12pt8b.bo.base_virt, DejaVuSerif12pt8b.bitmap, DejaVuSerif12pt8b.bitmap_size);
    hal_gfx_buffer_flush(&DejaVuSerif12pt8b.bo);
}
#endif

void load_objects(void)
{
    //Load memory objects
    fb.bo = hal_gfx_fb_create(fb.w*fb.h*PIXEL_DEPTH_BYTES);

    hal_gfx_buffer_map(&fb.bo);
    //printf("FB: V:%p P:0x%08lx\n", (void *)fb.bo.base_virt, fb.bo.base_phys);
}

static char fps_str[] = "FPS: 00.00 ";
void graphics_watch_demo_main(void)
{
    app_graphics_ospi_params_t params = PSRAM_INIT_PARAMS_Default;
    app_graphics_ospi_init(&params);

    graphics_gpu_init(NULL);
    graphics_dc_rm69330_qspi_lcd_init(LCD_RES_CURRENT, LCD_PIXEL_mode_24bit, GDC_MIPICFG_QSPI_RGB888_OPT0);
    app_graphics_mem_init((uint8_t*)GFX_MEM_BASE, GFX_MEM_SIZE);

    load_objects();

#if HAL_GDC_DISPLAY_ENABLE >0u
        s_dc_layer.resolution_x   = fb.w;
        s_dc_layer.resolution_y   = fb.h;

        s_dc_layer.data_format    = (graphics_dc_data_format_e)HAL_GFX_DC_FORMAT;
        s_dc_layer.blendmode      = HAL_GDC_BL_SRC;
        s_dc_layer.row_stride     = s_dc_layer.resolution_x*PIXEL_DEPTH_BYTES;
        s_dc_layer.start_x        = 0;
        s_dc_layer.start_y        = 0;
        s_dc_layer.size_x         = s_dc_layer.resolution_x;
        s_dc_layer.size_y         = s_dc_layer.resolution_y;
        s_dc_layer.alpha          = 0x00;

        s_dc_layer.frame_baseaddr = (void*)fb.bo.base_phys;
#endif

#if SIMULATION_WHICH == SIMULATION_454
    memcpy((void*)&s_watch_panel_img_cache, (void*)&s_imgs_watch_panel, sizeof(img_obj_t));
    s_watch_panel_img_cache.bo = hal_gfx_imagebuffer_create(s_imgs_watch_panel.bo.size);
    hal_gfx_buffer_map(&s_watch_panel_img_cache.bo);

    /* copy panel to cache */
    memcpy((void*)s_watch_panel_img_cache.bo.base_phys, (void*)&watch_panel_rgba_454p_1[0], sizeof(watch_panel_rgba_454p_1));
    memcpy((void*)((uint32_t)s_watch_panel_img_cache.bo.base_phys + sizeof(watch_panel_rgba_454p_1)), (void*)&watch_panel_rgba_454p_2[0], sizeof(watch_panel_rgba_454p_2));
    hal_gfx_buffer_flush(&s_watch_panel_img_cache.bo);

    /* copy panel to framebuffer */
    memcpy((void*)fb.bo.base_phys, (void*)&watch_panel_rgba_454p_1[0], sizeof(watch_panel_rgba_454p_1));
    memcpy((void*)((uint32_t)fb.bo.base_phys + sizeof(watch_panel_rgba_454p_1)), (void*)&watch_panel_rgba_454p_2[0], sizeof(watch_panel_rgba_454p_2));
#else
    memcpy((void*)&s_watch_panel_img_cache, (void*)&s_imgs_watch_panel, sizeof(img_obj_t));
    //s_watch_panel_img_cache.bo = hal_gfx_imagebuffer_create(s_imgs_watch_panel.bo.size);
    hal_gfx_buffer_map(&s_watch_panel_img_cache.bo);
    //memcpy((void*)s_watch_panel_img_cache.bo.base_phys, (void*)s_imgs_watch_panel.bo.base_phys, s_imgs_watch_panel.bo.size);
    hal_gfx_buffer_flush(&s_watch_panel_img_cache.bo);
    memcpy((void*)fb.bo.base_phys, (void*)s_imgs_watch_panel.bo.base_phys, s_imgs_watch_panel.bo.size);
#endif

    //load_font_object();

    int minx,miny,maxx,maxy;
    float start_time = hal_gfx_get_time();
    uint32_t frame = 0;

    do {
        hal_gfx_cmdlist_t cl  = hal_gfx_cl_le_create();  //Create Command Lists
        //Bind Command List
        hal_gfx_cl_bind(&cl);

        hal_gfx_bind_dst_tex(fb.bo.base_phys, fb.w, fb.h, fb.format, fb.stride);
        hal_gfx_set_clip(0, 0, RESX, RESY);
        hal_gfx_set_blend_blit(HAL_GFX_BL_SRC_OVER);
        //hal_gfx_bind_font(&DejaVuSerif12pt8b);
        //hal_gfx_string_get_bbox(fps_str, &font_w, &font_h, RESX, 1);

        hal_gfx_bind_src_tex(s_watch_panel_img_cache.bo.base_phys, s_watch_panel_img_cache.w, s_watch_panel_img_cache.h, s_watch_panel_img_cache.format, -1, HAL_GFX_FILTER_BL);
        hal_gfx_set_clip(minx, miny, maxx - minx, maxy - miny);
        hal_gfx_blit_rect((RESX - s_watch_panel_img_cache.w)/2, (RESY - s_watch_panel_img_cache.h)/2, s_watch_panel_img_cache.w, s_watch_panel_img_cache.h);   //scale to resx*resy

        hal_gfx_set_clip(0, 0, RESX, RESY);
        hal_gfx_bind_src_tex(s_imgs_watch_min_needle.bo.base_phys, s_imgs_watch_min_needle.w, s_imgs_watch_min_needle.h, s_imgs_watch_min_needle.format, -1, HAL_GFX_FILTER_BL);
        hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
        hal_gfx_clear_dirty_region();

        static float degree = 0;
        if (degree > 360)
        {
            degree = 0;
        }
        //hal_gfx_enable_aa(0,1,2,3);
        hal_gfx_set_blend_blit(HAL_GFX_BL_SRC_OVER);
        hal_gfx_blit_rotate_pivot(RESX/2, RESY/2, s_imgs_watch_min_needle.w - 20, s_imgs_watch_min_needle.h/2, degree);
        degree += 12;

        const uint32_t col = 0x55FF00;
        hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
        hal_gfx_print(fps_str, 0, 320, RESX, 32, col, HAL_GFX_ALIGNX_CENTER|HAL_GFX_TEXT_WRAP|HAL_GFX_ALIGNY_CENTER);

        hal_gfx_cl_submit(&cl);
        hal_gfx_cl_wait(&cl);

        hal_gfx_get_dirty_region(&minx, &miny, &maxx, &maxy);

        hal_gfx_cl_le_destroy(&cl);

#if HAL_GDC_DISPLAY_ENABLE > 0u
        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
        graphics_dc_rm69330_qspi_send_frame(s_dc_layer, DISPLAY_LCD_RES_X, DISPLAY_LCD_RES_Y);
#endif
        frame ++;
        hal_gfx_calculate_fps_ext(start_time, frame);
    } while (1);
}
