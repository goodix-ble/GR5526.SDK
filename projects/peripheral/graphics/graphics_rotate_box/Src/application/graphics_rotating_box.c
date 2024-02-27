#include "hal_gfx_core.h"
#include "hal_gfx_matrix4x4.h"
#include "hal_gfx_math.h"
#include "hal_gfx_utils.h"

#include "graphics_defs.h"
#include "graphics_dc_lcd_drv.h"

#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "app_graphics_mem.h"
#include "platform_sdk.h"

#include "image/graphics_rotate_box_rgba8888.h"

#if HAL_GDC_DISPLAY_ENABLE >0u
    #include "hal_gdc.h"
    #include "hal_gdc_mipi.h"
#endif


#define COLOR_CUBE      0       /* 1 - Color over CUBE; 0 - BOX image ober Cube */
#define FRAME_BUFFERS   1       /* Just Support 1 framebuffer for now, please don't change */

typedef enum {
    PRIM_LINE,
    PRIM_RECT,
    PRIM_TRI,
    PRIM_QUAD,
    PRIM_MAX
} primitive_e;

static img_obj_t s_gpu_fb[FRAME_BUFFERS];

#if HAL_GDC_DISPLAY_ENABLE >0u
    static TLS_VAR app_graphics_dc_framelayer_t s_dc_layer[FRAME_BUFFERS] = {(void *)0, SHOW_AREA_X, SHOW_AREA_Y, -1, 0, 0, SHOW_AREA_X, SHOW_AREA_Y, 0xff, HAL_GDC_BL_SRC, GDC_DATA_FORMAT_RGBA8888};
#endif

#if COLOR_CUBE == 0
    static img_obj_t s_box  = {{0}, 64, 64, -1, 0, HAL_GFX_FB_FORMAT, 0};
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
    int i;
    for (i = 0; i < FRAME_BUFFERS; ++i) {
        s_gpu_fb[i].w = SHOW_AREA_X;
        s_gpu_fb[i].h = SHOW_AREA_Y;
        s_gpu_fb[i].format = HAL_GFX_RGBA8888;
        s_gpu_fb[i].stride = SHOW_AREA_X*4;
        s_gpu_fb[i].bo = hal_gfx_fb_create(s_gpu_fb[i].stride*s_gpu_fb[i].h);
        hal_gfx_buffer_map(&s_gpu_fb[i].bo);

#if HAL_GDC_DISPLAY_ENABLE >0u
        s_dc_layer[i].resolution_x   = s_gpu_fb[i].w;
        s_dc_layer[i].resolution_y   = s_gpu_fb[i].h;

        s_dc_layer[i].data_format    = (graphics_dc_data_format_e)HAL_GFX_DC_FORMAT;
        s_dc_layer[i].blendmode      = HAL_GDC_BL_SRC;
        s_dc_layer[i].row_stride     = s_dc_layer[i].resolution_x*PIXEL_DEPTH_BYTES;
        s_dc_layer[i].start_x        = 0;
        s_dc_layer[i].start_y        = 0;
        s_dc_layer[i].size_x         = s_dc_layer[i].resolution_x;
        s_dc_layer[i].size_y         = s_dc_layer[i].resolution_y;
        s_dc_layer[i].alpha          = 0x00;

        s_dc_layer[i].frame_baseaddr = (void*)s_gpu_fb[i].bo.base_phys;
#endif
        printf("FB: V:%p P:0x%08x\n", (void *)s_gpu_fb[i].bo.base_virt, s_gpu_fb[i].bo.base_phys);
    }

#if COLOR_CUBE == 0
    s_box.bo           = hal_gfx_buffer_create(sizeof(test_img_64x64_rgba8888));
    s_box.bo.base_virt = (void *)test_img_64x64_rgba8888;
    s_box.bo.base_phys = (uintptr_t)test_img_64x64_rgba8888;
    s_box.bo.size      = sizeof(test_img_64x64_rgba8888);
#endif
}

static void draw_cube_side(float *v, int v0, int v1, int v2, int v3, uint32_t col) {
#if COLOR_CUBE
    //fill with color
    hal_gfx_fill_quad(v[v0*3], v[v0*3+1],
                   v[v1*3], v[v1*3+1],
                   v[v2*3], v[v2*3+1],
                   v[v3*3], v[v3*3+1], col);
#else
    //blit with box image
    hal_gfx_blit_quad_fit(v[v0*3], v[v0*3+1],
                   v[v1*3], v[v1*3+1],
                   v[v2*3], v[v2*3+1],
                   v[v3*3], v[v3*3+1]);

#endif
}

static void innercube(int angle_x, int angle_y, int angle_z)
{
    float box_size_2 = 0.2f;
    float FoV = 28.0724869359f;

                   //x     y    z
    float v[]   = {-box_size_2, -box_size_2, box_size_2,   //0  0
                    box_size_2, -box_size_2, box_size_2,   //1  3
                    box_size_2,  box_size_2, box_size_2,   //2  6
                   -box_size_2,  box_size_2, box_size_2,   //3  9
                   -box_size_2, -box_size_2,-box_size_2,   //4  12
                    box_size_2, -box_size_2,-box_size_2,   //5  15
                    box_size_2,  box_size_2,-box_size_2,   //6  18
                   -box_size_2,  box_size_2,-box_size_2};  //7  21

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, (float)SHOW_AREA_X/SHOW_AREA_Y, 0.2f, 100.f);

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);
    hal_gfx_mat4x4_rotate_X(proj, angle_x);
    hal_gfx_mat4x4_rotate_Y(proj, angle_y);
    hal_gfx_mat4x4_rotate_Z(proj, angle_z);
    hal_gfx_mat4x4_translate(proj, 0, 0, 2.f-box_size_2);

    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    int i;

    for (i = 0; i < 24; i+=3) {
        float w = 1.f;
        hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, SHOW_AREA_X, SHOW_AREA_Y,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    //blend color with background
#if COLOR_CUBE > 0
    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE);
#else
    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);
    hal_gfx_bind_src_tex(s_box.bo.base_phys, s_box.w, s_box.h, s_box.format, s_box.stride, HAL_GFX_FILTER_BL);
#endif

    //remove this to draw back sides also
    hal_gfx_tri_cull(HAL_GFX_CULL_CW);
    draw_cube_side(v, 0, 1, 2, 3, 0x60ffffff); //front
    draw_cube_side(v, 4, 0, 3, 7, 0x600000ff); //left
    draw_cube_side(v, 1, 5, 6, 2, 0x60ff00ff); //right
    draw_cube_side(v, 4, 5, 1, 0, 0x60ff0000); //top
    draw_cube_side(v, 3, 2, 6, 7, 0x6000ff00); //bottom
    draw_cube_side(v, 5, 4, 7, 6, 0x60808080); //back
    hal_gfx_tri_cull(HAL_GFX_CULL_NONE);
}

static int          cur_fb = 0;
static uintptr_t    cur_fb_base_phys = 0;

#if FRAME_BUFFERS > 1
static void swap_buffers(void)
{
#if HAL_GDC_DISPLAY_ENABLE >0u
    hal_gdc_wait_vsync();
    //hal_gdc_set_layer(0, &s_dc_layer[cur_fb]);
#endif
    cur_fb = (cur_fb+1)%FRAME_BUFFERS;
    cur_fb_base_phys = s_gpu_fb[cur_fb].bo.base_phys;
}
#endif

hal_gfx_cmdlist_t cl;
hal_gfx_cmdlist_t cl_clear;

static void hal_gfx_gfx_display_function(){
    static int angle_x = 0.f;
    static int angle_y = 0.f;
    static int angle_z = 80.f;

    angle_x = (angle_x+1)%360;
    angle_y = (angle_y+2)%360;
    //angle_z = (angle_z+2)%360;

    hal_gfx_cl_bind(&cl_clear);
    hal_gfx_cl_rewind(&cl_clear);

    hal_gfx_set_clip(0, 0, SHOW_AREA_X, SHOW_AREA_Y);
    hal_gfx_bind_dst_tex(cur_fb_base_phys, s_gpu_fb[0].w, s_gpu_fb[0].h, s_gpu_fb[0].format, s_gpu_fb[0].stride);

    hal_gfx_clear(0);
    hal_gfx_cl_submit(&cl_clear);

    hal_gfx_cl_bind(&cl);
    hal_gfx_cl_rewind(&cl);

    hal_gfx_set_clip(0, 0, SHOW_AREA_X, SHOW_AREA_Y);
    hal_gfx_bind_dst_tex(cur_fb_base_phys, s_gpu_fb[0].w, s_gpu_fb[0].h, s_gpu_fb[0].format, s_gpu_fb[0].stride);

    innercube(angle_x, angle_y, angle_z);

    hal_gfx_cl_submit(&cl);
    hal_gfx_cl_wait(&cl);
#if FRAME_BUFFERS > 1
    swap_buffers();
#endif
}

extern float hal_gfx_calculate_fps_ext(float start_time, uint32_t frame) ;

int graphics_rotating_box(void)
{
    app_graphics_ospi_params_t params = PSRAM_INIT_PARAMS_Default;
    app_graphics_ospi_init(&params);

    graphics_gpu_init(NULL);
    graphics_dc_rm69330_qspi_lcd_init(LCD_RES_CURRENT, LCD_PIXEL_mode_24bit, GDC_MIPICFG_QSPI_RGB888_OPT0);
    app_graphics_mem_init((uint8_t*)GFX_MEM_BASE, GFX_MEM_SIZE);

    load_objects();
    cur_fb_base_phys = s_gpu_fb[cur_fb].bo.base_phys;

    //Create Command Lists
    cl  = hal_gfx_cl_le_create();
    cl_clear = hal_gfx_cl_le_create();

    uint32_t frame = 0;
    float start_time = hal_gfx_get_time();

    while(1){
        hal_gfx_gfx_display_function();
#if HAL_GDC_DISPLAY_ENABLE > 0u
        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
        graphics_dc_rm69330_qspi_send_frame(s_dc_layer[0], DISPLAY_LCD_RES_X, DISPLAY_LCD_RES_Y);
#endif
        frame ++;
        hal_gfx_calculate_fps_ext(start_time, frame);
    }

#pragma diag_suppress   111
    hal_gfx_cl_le_destroy(&cl);
    hal_gfx_cl_le_destroy(&cl_clear);
    app_graphics_mem_free(s_gpu_fb[0].bo.base_virt);

    return 0;
}
