#include "lv_obj.h"
#include "lv_indev.h"
#include "lv_refr.h"
#include "lv_group.h"
#include "lv_disp.h"
#include "lv_theme.h"
#include "app_log.h"
#include "app_graphics_mem.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"
#include "disp_driver.h"
#include "hal_gfx_programHW.h"
#include "hal_gfx_regs.h"
#include "hal_gfx_ringbuffer.h"
#include "hal_gfx_rasterizer.h"
#include "hal_gfx_raster.h"
#include "grx_hal.h"
#include "app_graphics_mem.h"
#include <math.h>
#include "image_left_rgba565.h"
#include "image_left_mirror_tsc4.h"
#include "lv_img_dsc_list.h"
#include "lv_port_disp.h"
#include "hal_gfx_graphics_enhance.h"
#include "lv_graphics_draw.h"
/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_graphics_draw_class
#define CANVAS_BUF_SIZE     (1*(454*454*2))

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_obj_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_obj_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_obj_draw(lv_event_t * e);
static void lv_obj_event(const lv_obj_class_t * class_p, lv_event_t * e);


static void _free_canvas_draw_circles(void);
static void _free_canvas_draw_perodic_arcs(void);
static void _free_canvas_draw_stencil(void);
static void _free_canvas_draw_incube_box(void);
static void _free_canvas_draw_jumping_ball(void);
static void _free_canvas_draw_tsc4_comp(void);


/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_graphics_draw_class = {
    .constructor_cb = lv_obj_constructor,
    .destructor_cb = lv_obj_destructor,
    .event_cb = lv_obj_event,
    .instance_size = (sizeof(lv_obj_t)),
    .base_class = &lv_obj_class,
};

static void* s_canvas_buf = NULL;
static int step = 0;
static int step_count = 0;
static bool is_box_image_cache = false;
static bool is_prepared_shape = false;
static free_graphics_type_e draw_type;
static lv_obj_t* cur_obj;
static lv_timer_t* draw_timer;
/**********************
 *      DECLARATIONS
 **********************/
void lv_port_res_mode_set(uint8_t mode);
extern hal_gfx_cmdlist_t * lv_port_get_current_cl(void);
/**********************
 *   GLOBAL FUNCTIONS
 **********************/


lv_obj_t * lv_graphics_draw_obj_create(lv_obj_t * parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    cur_obj = obj;
    return obj;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
void lv_draw_graphics_timer(lv_timer_t * tmr)
{
    lv_obj_invalidate(cur_obj);
}
static void lv_obj_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_t * parent = obj->parent;
    obj->flags = LV_OBJ_FLAG_CLICKABLE;
    obj->flags |= LV_OBJ_FLAG_SNAPPABLE;
    obj->flags |= LV_OBJ_FLAG_SCROLLABLE;
    if(parent) obj->flags |= LV_OBJ_FLAG_PRESS_LOCK;
    if (s_canvas_buf == NULL){
        s_canvas_buf = app_graphics_mem_malloc(CANVAS_BUF_SIZE);    
    }
    step = 0;
    step_count = 0;
    is_prepared_shape = false;
    is_box_image_cache = false;
    if (draw_timer == NULL)
    {
        draw_timer = lv_timer_create(lv_draw_graphics_timer, 20, NULL);
    }
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_obj_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    if(NULL != s_canvas_buf)
    {
        app_graphics_mem_free(s_canvas_buf);
        //printf("canvas buf destroy: %08X\n", (uint32_t)s_canvas_buf);
        s_canvas_buf = NULL;
    }
    step = 0;
    step_count = 0;
    is_prepared_shape = false;
    is_box_image_cache = false;
    if (draw_timer != NULL)
    {
        lv_timer_del(draw_timer);
        draw_timer = NULL;
    }
}


void lv_set_graphics_draw_type(free_graphics_type_e type) {
    draw_type = type;
}

static void lv_obj_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_DRAW_MAIN) {
        lv_obj_draw(e);
    }
}

static void lv_obj_draw(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    switch (draw_type) {

        case GRAPHICS_TYPE_CIRCLE:
        {
            step ++;
            if(step == 5) {
                step = 0;
                step_count ++;
            } else {
                _free_canvas_draw_circles();
                return;
            }

            //if(step_count < 2000)
            {
                _free_canvas_draw_circles();
            }
        }
        break;

        case GRAPHICS_TYPE_ARCS:
        {
            step ++;
            if(step == 3) {
                step = 0;
                step_count ++;
            } else {
                _free_canvas_draw_perodic_arcs();
                return;
            }

            //if(step_count < 15000)
            {
                _free_canvas_draw_perodic_arcs();
            }
        }
        break;

        case GRAPHICS_TYPE_STENCIL:
        {
            step ++;
            if(step == 1) {
                step = 0;
                step_count ++;
            } else {
                _free_canvas_draw_stencil();
                return;
            }

            //if(step_count < 3000)
            {
                _free_canvas_draw_stencil();
            }
        }
        break;

        case GRAPHICS_TYPE_ROTATE_CUBE:
        {
            step ++;
            if(step == 1) {
                step = 0;
                step_count ++;
            } else {
                _free_canvas_draw_incube_box();
                return;
            }

            //if(step_count < 100000)
            {
                _free_canvas_draw_incube_box();
            }
        }
        break;

        case GRAPHICS_TYPE_JUMPING_SHAPE:
        {
            if (step == 0 && step_count == 0)
            {
                _free_canvas_draw_jumping_ball();
                step++;
            }else{
                step ++;
                if(step == 3) {
                    step = 0;
                    step_count ++;
                } else {
                    
                    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
                    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
                    if (draw_buf->buf_act == draw_buf->buf1){
                        memcpy(draw_buf->buf_act, draw_buf->buf2, draw_buf->size * 2);
                    }else{
                        memcpy(draw_buf->buf_act, draw_buf->buf1, draw_buf->size * 2);
                    }
                    return;
                }
                //if(step_count < 100000)
                {
                    _free_canvas_draw_jumping_ball();
                }
            }
        }
        break;


        case GRAPHICS_TYPE_TSC4_COMP:
        {
            step ++;
            if(step == 20) {
                step = 0;
                step_count ++;
            } else {
                _free_canvas_draw_tsc4_comp();
                return;
            }

            {
                _free_canvas_draw_tsc4_comp();
            }
        }
        break;
    }

    return;
}

//static char * get_show_tips(uint32_t id)
//{
//    return (char*)"RGB565::TSC4";
//}

/**********************************************************/

typedef struct {
    float x;
    float y;
} _coord_t;

static _coord_t s_cirle_coords[8] = {
        {113, 0},  {0, -113}, {-113,0}, {0,  113},
        {79.9, -79.9},  {-79.9, -79.9},  {-79.9, 79.9},  {79.9, 79.9},
    };

static uint32_t s_color[] = {0x80ffb6c1, 0x80FFff00, 0x80c71585, 0x80ff4500};

static void _free_canvas_draw_circles() {

    int count = step_count % 32;
    void * p_cache = s_canvas_buf;
    
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;


    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_bind_dst_tex((uint32_t)p_cache, ver_res, hor_res, HAL_GFX_RGB565, -1);

    if(step_count <= 1) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE);
        hal_gfx_fill_rect(0,0,454,454, 0xffffffff);
        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);
    }

    count = step_count % 16;

    if(count < 4) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE  | HAL_GFX_BLOP_MODULATE_A );
        hal_gfx_set_const_color(s_color[0]);
        hal_gfx_fill_circle_aa(227 + s_cirle_coords[count%8].x, 227 + s_cirle_coords[count%8].y, 113, s_color[0]);
    } else if(count < 8) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE  | HAL_GFX_BLOP_MODULATE_A );
        hal_gfx_set_const_color(s_color[1]);
        hal_gfx_fill_circle_aa(227 + s_cirle_coords[count%8].x, 227 + s_cirle_coords[count%8].y, 113, s_color[1]);
    } else if(count < 12) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE  | HAL_GFX_BLOP_MODULATE_A );
        hal_gfx_set_const_color(s_color[2]);
        hal_gfx_fill_circle_aa(227 + s_cirle_coords[count%8].x, 227 + s_cirle_coords[count%8].y, 113, s_color[2]);
    } else if(count < 16) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE  | HAL_GFX_BLOP_MODULATE_A );
        hal_gfx_set_const_color(s_color[3]);
        hal_gfx_fill_circle_aa(227 + s_cirle_coords[count%8].x, 227 + s_cirle_coords[count%8].y, 113, s_color[3]);
    }

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

#if 1

    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, ver_res, hor_res, HAL_GFX_RGB565, -1);
    hal_gfx_bind_src_tex((uint32_t)p_cache, ver_res, hor_res, HAL_GFX_RGB565, -1, HAL_GFX_FILTER_PS);
    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);
    hal_gfx_blit(0, 0);
    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);
#endif
}

/**********************************************************/

const static uint32_t s_arc_width[6] = {
    20, 25, 30, 35, 40, 50
    };

const static uint32_t s_arc_count[6] = {
    10, 8, 7, 6, 5, 4
    };

const static uint32_t s_degree_delta[10] = {
    10, 15, 20, 25, 30, 35, 40, 45, 50, 55,
    };
const static uint32_t s_color_table[30] = {
    0xffffc0cb,
    0xffdb7093,
    0xffff00ff,
    0xffda70d6,
    0xff8a2be2,
    0xff9932cc,
    0xff1e90ff,
    0xff7fffaa,
    0xff008080,
    0xff00ffff,
    0xff3cb371,
    0xff90ee90,
    0xffadff2f,
    0xffffff00,
    0xffffd700,
    0xfff08080,
    0xffff6347,
    0xffd2691e,
    0xffffa500,
    0xff808000,
    0xffff8c00,
    0xffadff2f,
    0xff32cd32,
    0xff4169e1,
    0xff00ff00,
    0xffdbd76b,
    0xffdaa520,
    0xff00fa9a,
    0xfff0e68c,
    0xffee82ee,
};

static uint32_t s_arc_level;

//extern void hal_gfx_draw_arc_x(float x, float y, float r, float w, uint16_t start_angle, uint16_t end_angle, bool is_rounded, uint32_t rgba8888);

static void _free_canvas_draw_perodic_arcs() {

    uint16_t count = step_count;

    const uint32_t  arc1_start = 0,
                    arc1_end   = 50,
                    arc2_start = 120,
                    arc2_end   = 170,
                    arc3_start = 240,
                    arc3_end   = 290;

    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;
    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, ver_res, hor_res, HAL_GFX_RGB565, -1);
    s_arc_level = (count/30);
    s_arc_level = s_arc_level % 6;

    uint32_t arc_cnt   = s_arc_count[s_arc_level];
    uint32_t arc_width = s_arc_width[s_arc_level];
    uint32_t degree_delta;

    uint32_t i;
    uint32_t r_base =10;

    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE);
    hal_gfx_fill_rect(0,0,454,454, 0xffffffff);
    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE);
    for(i = 0; i < arc_cnt; i++) {
        degree_delta = s_degree_delta[i];
        hal_gfx_draw_arc(227, 227, r_base + (i+1)*arc_width, arc_width,
                                   (arc1_start + count*degree_delta) % 720, (arc1_end + count*degree_delta) % 720,
                                   false, s_color_table[3*i]);

        hal_gfx_draw_arc(227, 227, r_base + (i+1)*arc_width, arc_width,
                                   (arc2_start + count*degree_delta) % 720, (arc2_end + count*degree_delta) % 720,
                                   false, s_color_table[3*i +1]);

        hal_gfx_draw_arc(227, 227, r_base + (i+1)*arc_width, arc_width,
                                   (arc3_start + count*degree_delta) % 720, (arc3_end + count*degree_delta) % 720,
                                   false, s_color_table[3*i+2]);
        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);
    }

}

/**********************************************************/

#include "gx_stencil_img.h"


static void _free_canvas_draw_stencil() {

    uint16_t count = step_count;

    const int x = 128;

    int y_up_start   = -192;
    int y_down_start = 448;
    int y_delta = 8 ; //320 / 40 = 8

    count = count % 60;


    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, ver_res, hor_res, HAL_GFX_RGB565, -1);
    hal_gfx_set_clip(0, 0, 454, 454);

    if(count < 40) {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
        hal_gfx_fill_rect(0,0,454,454, 0xffffffff);
        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);

     //   hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A);

        hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
       // hal_gfx_set_const_color(0x8000ff00);

        lv_port_res_mode_set(2);

        hal_gfx_bind_src_tex((uintptr_t)ADDR_STENCIL_MASK_200P+ QSPI0_XIP_BASE, 200, 200, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_PS);
        hal_gfx_blit(x, y_up_start + count*y_delta);

        hal_gfx_bind_src_tex((uintptr_t)ADDR_STENCIL_LOGO_200P+ QSPI0_XIP_BASE, 200, 200, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_PS);
        hal_gfx_blit(x, y_down_start - count*y_delta);
        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);

    } else {
        hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
        hal_gfx_fill_rect(0,0,454,454, 0xffffffff);
        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);

        lv_port_res_mode_set(2);
        hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_STENCIL_TXTY  | HAL_GFX_BLOP_DST_CKEY_NEG, HAL_GFX_TEX0, HAL_GFX_TEX1, HAL_GFX_NOTEX );
        hal_gfx_bind_src_tex((uintptr_t)ADDR_STENCIL_MASK_200P+ QSPI0_XIP_BASE, 200, 200, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_PS);
        hal_gfx_bind_depth_buffer((uintptr_t)ADDR_STENCIL_LOGO_200P+ QSPI0_XIP_BASE, 200, 200);
        //hal_gfx_bind_tex(HAL_GFX_TEX3, (uintptr_t)stencil_logo_rgba565, 300, 80, HAL_GFX_RGB565, -1, HAL_GFX_FILTER_PS);

        hal_gfx_set_dst_color_key(0xff00ff00);

        hal_gfx_blit_rect (x, 128, 200, 200);

        hal_gfx_cl_submit(cmd);
        hal_gfx_cl_wait(cmd);
    }
    lv_port_res_mode_set(1);
}

/**********************************************************/
#define COLOR_CUBE      1

static void _draw_cube_side(float *v, int v0, int v1, int v2, int v3, uint32_t col) {
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

static void _draw_cube_side_fit(float *v, int v0, int v1, int v2, int v3) {
    //blit with box image
    hal_gfx_blit_quad_fit(v[v0*3], v[v0*3+1],
                   v[v1*3], v[v1*3+1],
                   v[v2*3], v[v2*3+1],
                   v[v3*3], v[v3*3+1]);

}

static void _innercube(int angle_x, int angle_y, int angle_z)
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

    hal_gfx_mat4x4_load_perspective(mvp, FoV, (float)454/454, 0.2f, 100.f);

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);
    hal_gfx_mat4x4_rotate_X(proj, angle_x);
    hal_gfx_mat4x4_rotate_Y(proj, angle_y);
    hal_gfx_mat4x4_rotate_Z(proj, angle_z);
    hal_gfx_mat4x4_translate(proj, 0, 0, 2.f-box_size_2);

    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    int i;

    void * pic = s_canvas_buf;

    for (i = 0; i < 24; i+=3) {
        float w = 1.f;
        hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, 454, 454,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    if(!is_box_image_cache) {
        lv_port_res_mode_set(2);
        memcpy((void*)pic, (void*)(QSPI0_XIP_BASE + ADDR_GOODIX_LOGO), 150*150*4);
        is_box_image_cache = true;
        lv_port_res_mode_set(1);
    }

    //blend color with background
#if COLOR_CUBE > 0
    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE );

    //hal_gfx_set_const_color(0xff000000);
#else

    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);
    hal_gfx_bind_src_tex((uintptr_t)pic, 150, 150, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_BL);
#endif

    static int cnt = 0;

    //remove this to draw back sides also
    hal_gfx_tri_cull(HAL_GFX_CULL_NONE);
    _draw_cube_side(v, 0, 1, 2, 3, 0xA08b008b); //front
    _draw_cube_side(v, 4, 0, 3, 7, 0xA0228b22); //left
    _draw_cube_side(v, 1, 5, 6, 2, 0xA0ff4500); //right
    _draw_cube_side(v, 4, 5, 1, 0, 0xA0003366); //top
    _draw_cube_side(v, 3, 2, 6, 7, 0xA066cc99); //bottom
#if 0
    _draw_cube_side(v, 5, 4, 7, 6, 0x60808080); //back
#else
    hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
    hal_gfx_bind_src_tex((uintptr_t)pic, 80, 80, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_BL);
    _draw_cube_side_fit(v, 5, 4, 7, 6);
#endif
    hal_gfx_tri_cull(HAL_GFX_CULL_NONE);

    cnt++;

    cnt = cnt % 24;
}

static void _free_canvas_draw_incube_box() {


    static int angle_x = 0.f;
    static int angle_y = 0.f;
    static int angle_z = 80.f;

    angle_x = (angle_x+3)%360;
    angle_y = (angle_y+3)%360;
    angle_z = (angle_z+3)%360;


    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;
    
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, ver_res, hor_res, HAL_GFX_RGB565, -1);
    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_clear(0);
    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

    _innercube(angle_x, angle_y, angle_z);

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);
}


/**********************************************************/\
// Jumping Ball

static void * _prepare_jumping_shape(hal_gfx_cmdlist_t * cl, int shape) {
    void * buff = s_canvas_buf;

    /* 80*80*4 */
    switch(shape) {
        case 0: // rect
        {
            hal_gfx_bind_dst_tex((uint32_t)buff, 80, 80, HAL_GFX_RGBA8888, -1);
            hal_gfx_set_clip(0, 0, 80, 80);
            hal_gfx_clear(0);
            hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
            hal_gfx_fill_rounded_rect_aa(0, 0, 80, 80, 10, 0xff32cd32);
            hal_gfx_cl_submit(cl);
            hal_gfx_cl_wait(cl);
        }
        break;

        case 1: // triangle
        {
            buff = (void*)((uint32_t)buff + 0xD000);
            hal_gfx_bind_dst_tex((uint32_t)buff, 80, 80, HAL_GFX_RGBA8888, -1);
            hal_gfx_set_clip(0, 0, 80, 80);
            hal_gfx_clear(0);
            hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
            hal_gfx_fill_triangle(40, 0, 80, 80, 0, 80, 0xff0000ff);
            hal_gfx_cl_submit(cl);
            hal_gfx_cl_wait(cl);
        }
        break;

        case 2: // circle
        default:
        {
            buff = (void*)((uint32_t)buff + 0xD000*2);
            hal_gfx_bind_dst_tex((uint32_t)buff, 80, 80, HAL_GFX_RGBA8888, -1);
            hal_gfx_set_clip(0, 0, 80, 80);
            hal_gfx_clear(0);
            hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
            //hal_gfx_fill_circle_aa(40, 40, 40, 0xff9400d3);
            hal_gfx_draw_arc(40,40,40,40, 0,360,false, 0xff9400d3);
            hal_gfx_cl_submit(cl);
            hal_gfx_cl_wait(cl);
        }
        break;
    }


    return buff;
}

#define  _STEP  14

static uint16_t _y_path_delta_up[_STEP] = {0,10,20,30,40,40,30,20,20,20,10,10, 10, 0};
static uint16_t _angle_rotate_up[_STEP] = {0,0, 0, 0, 0, 0, 0, 0, 0, 0 ,300,240,180,180};

static uint16_t _y_path_delta_down[_STEP] = {0,  5,  10, 15, 15, 20,20,25,25,30,30,35,30,0};
static uint16_t _angle_rotate_down[_STEP] = {150,120,60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0,0};

static uint16_t _base_width[_STEP] = {140,134,130,120,110,100,90,80,70,60,50,40,30,20};

static float _ball_scale[_STEP] = {0.95,0.95,0.96,0.965,0.97,0.975,0.98,0.985,0.99,0.995,1.0,1.05,1.05,1.07};

static void _render_jumping_shape(uint32_t count) {
    const int  x = 184;
    const static int y_start = 320;
    static int y = y_start;

    uint32_t cnt = count%(2*_STEP);

    if(cnt == 0) {
        y = y_start;
    }

    if(cnt < _STEP) {
        y -= _y_path_delta_up[cnt];

        if(_angle_rotate_up[cnt] != 0) {
            hal_gfx_blit_rotate_pivot(x+40,y+40, 40,40, _angle_rotate_up[cnt]);
        } else {
            hal_gfx_blit(x, y);
        }
    } else {
        y += _y_path_delta_down[cnt-_STEP];

        if(_angle_rotate_down[cnt-_STEP] != 0) {
            hal_gfx_blit_rotate_pivot(x+40, y+40, 40,40, _angle_rotate_down[cnt-_STEP]);
        } else {
            hal_gfx_blit(x, y);
        }

    }

    return;
}

static void _render_jumping_ball(uint32_t count) {
    const int  x = 184;
    const static int y_start = 320;
    static int y = y_start;

    uint32_t cnt = count%(2*_STEP);

    if(cnt == 0) {
        y = y_start;
    }

    if(cnt < _STEP) {
        y -= _y_path_delta_up[cnt];
        hal_gfx_blit_rotate_pivot_scale(x+40, y+40, 40, 40, _angle_rotate_up[cnt], _ball_scale[cnt]);

    } else {
        y += _y_path_delta_down[cnt-_STEP];
        hal_gfx_blit_rotate_pivot_scale(x+40, y+40, 40, 40, _angle_rotate_down[cnt-_STEP], _ball_scale[2*_STEP -1 - cnt]);
    }

    return;
}

static void _render_base_shape(uint32_t count) {
    uint32_t cnt = count%(2*_STEP);

    if(cnt < _STEP) {
        hal_gfx_fill_rect((454 - _base_width[cnt])/2, 404, _base_width[cnt], 8, 0xA0003366);
    } else {
        hal_gfx_fill_rect((454 - _base_width[2*_STEP -1 - cnt])/2, 404, _base_width[2*_STEP -1 - cnt], 8, 0xA0003366);
    }
}

static void * p_rect_shape = NULL;
static void * p_triangle_shape = NULL;
static void * p_circle_shape = NULL;
#define     _STEP  14

static void _free_canvas_draw_jumping_ball() {


    uint32_t count = step_count % (6*_STEP);

    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

//    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
//    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;

    if(!is_prepared_shape) {
        p_rect_shape        = _prepare_jumping_shape(cmd, 0);
        p_triangle_shape    = _prepare_jumping_shape(cmd, 1);
        p_circle_shape      = _prepare_jumping_shape(cmd, 2);
        is_prepared_shape   = true;
    }

    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, 454, 454, HAL_GFX_RGB565, -1);
    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_clear(0);
    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);

    hal_gfx_enable_aa(1,1,1,1);

    if(count < 2*_STEP) {
        hal_gfx_bind_src_tex((uintptr_t)p_rect_shape, 80, 80, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_PS);
        _render_jumping_shape(step_count);
    } else if (count < 4*_STEP) {
        hal_gfx_bind_src_tex((uintptr_t)p_triangle_shape, 80, 80, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_PS);
        _render_jumping_shape(step_count);
    } else {
        hal_gfx_bind_src_tex((uintptr_t)p_circle_shape, 80, 80, HAL_GFX_RGBA8888, -1, HAL_GFX_FILTER_BL);
        _render_jumping_ball(step_count);
    }

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

    hal_gfx_set_blend_fill(HAL_GFX_BL_SIMPLE);
    _render_base_shape(step_count);

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

}

static void _free_canvas_draw_tsc4_comp() {

    uint8_t * p_tsc4 = app_graphics_mem_malloc(image_left_mirror_tsc4_length);

    if(p_tsc4 != NULL) {
        memcpy(p_tsc4, image_left_mirror_tsc4, image_left_mirror_tsc4_length);
    }

    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t * cmd = lv_port_get_current_cl();

//    uint16_t ver_res = draw_buf->area.y2 - draw_buf->area.y1 + 1;
//    uint16_t hor_res = draw_buf->area.x2 - draw_buf->area.x1 + 1;

    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, 454, 454, HAL_GFX_RGB565, -1);
    hal_gfx_set_clip(0, 0, 454, 454);
    hal_gfx_clear(0);
    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);

    hal_gfx_bind_src_tex((uint32_t)image_left_rgba565, 224, 452, HAL_GFX_RGBA5650, -1, HAL_GFX_FILTER_BL);
    hal_gfx_blit(2, 0);

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

    if(p_tsc4 != NULL) {
        hal_gfx_bind_src_tex((uint32_t)p_tsc4, 224, 452, HAL_GFX_TSC4, -1, HAL_GFX_FILTER_BL);
    } else {
        hal_gfx_bind_src_tex((uint32_t)image_left_mirror_tsc4, 224, 452, HAL_GFX_TSC4, -1, HAL_GFX_FILTER_BL);
    }

    hal_gfx_blit(228, 0);

    hal_gfx_cl_submit(cmd);
    hal_gfx_cl_wait(cmd);

    if(p_tsc4 != NULL) {
        app_graphics_mem_free(p_tsc4);
    }
}

