#include "string.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_font.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"

#include "graphics_defs.h"
#include "graphics_dc_lcd_drv.h"

#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "app_graphics_mem.h"
#include "platform_sdk.h"

#pragma diag_suppress   1296

#if HAL_GDC_DISPLAY_ENABLE > 0u
    #include "hal_gdc.h"
    #include "hal_gdc_mipi.h"
#endif


#define ANIMATION_STEP_0_1      0.06f

#if SHOW_AREA_X >= 390
    #include "image/watch_img_1_388p_tsc6a.h"
    #include "image/watch_img_2_388p_tsc6a.h"

    #define THIS_IMG_W       388
    #define THIS_IMG_H       388

    static img_obj_t screen0 = {
                            {.size = watch_img_1_388p_tsc6a_length,
                             .base_virt = (void *)watch_img_1_388p_tsc6a,
                             .base_phys = (uintptr_t)watch_img_1_388p_tsc6a},
                             THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_TSC6A, 0};
    static img_obj_t screen1 = {
                            {.size = watch_img_2_388p_tsc6a_length,
                             .base_virt = (void *)watch_img_2_388p_tsc6a,
                             .base_phys = (uintptr_t)watch_img_2_388p_tsc6a},
                             THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_TSC6A, 0};
#else

    #if TSCx_ENABLE > 0u
        #include "image/watch_img_1_240p_tsc6a.h"
        #include "image/watch_img_2_240p_tsc6a.h"

        #define THIS_IMG_W       240
        #define THIS_IMG_H       240

        static img_obj_t screen0 = {
                                {.size = watch_img_1_240p_tsc6a_length,
                                 .base_virt = (void *)watch_img_1_240p_tsc6a,
                                 .base_phys = (uintptr_t)watch_img_1_240p_tsc6a},
                                 THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_TSC6A, 0};
        static img_obj_t screen1 = {
                                {.size = watch_img_2_240p_tsc6a_length,
                                 .base_virt = (void *)watch_img_2_240p_tsc6a,
                                 .base_phys = (uintptr_t)watch_img_2_240p_tsc6a},
                                 THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_TSC6A, 0};
    #else
        #include "image/watch_img_1_240p_rgba8888.h"
        #include "image/watch_img_2_240p_rgba8888.h"

        #define THIS_IMG_W       240
        #define THIS_IMG_H       240

        static img_obj_t screen0 = {
                                {.size = watch_img_1_240p_rgba8888_length,
                                 .base_virt = (void *)watch_img_1_240p_rgba8888,
                                 .base_phys = (uintptr_t)watch_img_1_240p_rgba8888},
                                 THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_RGBA8888, 0};
        static img_obj_t screen1 = {
                                {.size = watch_img_2_240p_rgba8888_length,
                                 .base_virt = (void *)watch_img_2_240p_rgba8888,
                                 .base_phys = (uintptr_t)watch_img_2_240p_rgba8888},
                                 THIS_IMG_W, THIS_IMG_H, -1, 0, (uint8_t)HAL_GFX_RGBA8888, 0};
    #endif

#endif

static img_obj_t s_fb = {{0}, SHOW_AREA_X, SHOW_AREA_Y, -1, 0, HAL_GFX_RGBA8888, 0};

#if HAL_GDC_DISPLAY_ENABLE >0u
    static TLS_VAR app_graphics_dc_framelayer_t s_dc_layer = {(void *)0, SHOW_AREA_X, SHOW_AREA_Y, -1, 0, 0, SHOW_AREA_X, SHOW_AREA_Y, 0xff, HAL_GDC_BL_SRC, GDC_DATA_FORMAT_RGBA8888};
#endif


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

static void load_objects(void)
{
    s_fb.bo = hal_gfx_fb_create(s_fb.w*s_fb.h*PIXEL_DEPTH_BYTES);

    hal_gfx_buffer_map(&s_fb.bo);

#if HAL_GDC_DISPLAY_ENABLE >0u
    s_dc_layer.resolution_x   = s_fb.w;
    s_dc_layer.resolution_y   = s_fb.h;
    #if TSCx_ENABLE > 0u
        s_dc_layer.data_format    = GDC_DATA_FORMAT_RGBA8888 ;//| HAL_GDC_MODULATE_A;
    #else
        s_dc_layer.data_format    = GDC_DATA_FORMAT_RGBA8888 ;
    #endif
    s_dc_layer.row_stride     = s_dc_layer.resolution_x*PIXEL_DEPTH_BYTES;
    s_dc_layer.blendmode      = HAL_GDC_BL_SIMPLE;
    s_dc_layer.start_x        = 0;
    s_dc_layer.start_y        = 0;
    s_dc_layer.size_x         = s_dc_layer.resolution_x;
    s_dc_layer.size_y         = s_dc_layer.resolution_y;
    s_dc_layer.alpha          = 0x80;
    s_dc_layer.frame_baseaddr = (void*)s_fb.bo.base_phys;
#endif

    printf("FB: V:%p P:0x%08x\n", (void *)s_fb.bo.base_virt, (uint32_t)s_fb.bo.base_phys);
}

static hal_gfx_cmdlist_t cl;

static int init(void)
{
    app_graphics_ospi_params_t params = PSRAM_INIT_PARAMS_Default;
    app_graphics_ospi_init(&params);

    graphics_gpu_init(NULL);
    graphics_dc_rm69330_qspi_lcd_init(LCD_RES_CURRENT, LCD_PIXEL_mode_24bit, GDC_MIPICFG_QSPI_RGB888_OPT0);

    app_graphics_mem_init((uint8_t*)GFX_MEM_BASE, GFX_MEM_SIZE);

    load_objects();

    //Create Command Lists
    cl  = hal_gfx_cl_le_create();

    //Bind Command List
    hal_gfx_cl_bind(&cl);

    //Set Clipping Rectangle
    hal_gfx_set_clip(0, 0, SHOW_AREA_X, SHOW_AREA_Y);
#if 0
    hal_gfx_bind_tex(HAL_GFX_TEX1, screen0.bo.base_phys,  \
        screen0.w, screen0.h, (hal_gfx_tex_format_t)(screen0.format), screen0.stride, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER));
    hal_gfx_bind_tex(HAL_GFX_TEX2, screen1.bo.base_phys,  \
        screen1.w, screen1.h, (hal_gfx_tex_format_t)(screen1.format), screen1.stride, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER));
#else
    hal_gfx_bind_src_tex(screen0.bo.base_phys,  \
        screen0.w, screen0.h, (hal_gfx_tex_format_t)(screen0.format), screen0.stride, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER ));
    hal_gfx_bind_src2_tex(screen1.bo.base_phys,  \
        screen1.w, screen1.h, (hal_gfx_tex_format_t)(screen1.format), screen1.stride, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER ));
#endif
    hal_gfx_set_tex_color(0);

    hal_gfx_cl_submit(&cl);
    hal_gfx_cl_wait(&cl);

    return 0;
}

static hal_gfx_transition_t s_effect = HAL_GFX_TRANS_STACK_H;

static void next_effect(void)
{
    s_effect = (hal_gfx_transition_t)(((int)s_effect + 1) % HAL_GFX_TRANS_MAX);
}

static void display(float step, uint32_t blendmode)
{
    hal_gfx_cl_rewind(&cl);

    //Bind Framebuffer
    hal_gfx_bind_dst_tex((uint32_t)s_fb.bo.base_phys, s_fb.w, s_fb.h, (hal_gfx_tex_format_t)(s_fb.format), s_fb.stride);

    hal_gfx_transition(s_effect, HAL_GFX_TEX1, HAL_GFX_TEX2, blendmode, step, s_fb.w, s_fb.h);

    hal_gfx_cl_submit(&cl);
    (void)hal_gfx_cl_wait(&cl);

#if HAL_GDC_DISPLAY_ENABLE > 0u
    app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    graphics_dc_rm69330_qspi_send_frame(s_dc_layer, DISPLAY_LCD_RES_X, DISPLAY_LCD_RES_Y);
#endif
}

extern float hal_gfx_calculate_fps_ext(float start_time, uint32_t frame) ;

static void animation_effects_loop(void)
{
    float step = 0.f;
    float step_step = ANIMATION_STEP_0_1;

    float start_time = hal_gfx_get_time();
    int frame = 0;

    while (1)
    {
        display(step, HAL_GFX_BL_SRC);

        if (step <= 0.f)
        {
            step = 0.f;
            step_step = ANIMATION_STEP_0_1;
            next_effect();
        }
        else if (step >= 1.f)
        {
            step = 1.f;
            step_step = -ANIMATION_STEP_0_1;
            next_effect();
        }

        step += step_step;

        if (step > 0.30f)
        {
            //step = 0.f;
        }
        frame++;
        hal_gfx_calculate_fps_ext(start_time, frame);
    }
}

int graphics_animation_effects_main(void)
{
    int ret = init();
    if (ret)
    {
        app_graphics_mem_free(s_fb.bo.base_virt);
        return ret;
    }

    animation_effects_loop();
    hal_gfx_cl_le_destroy(&cl);
    app_graphics_mem_free(s_fb.bo.base_virt);
    return 0;
}
