/**
 * @file lv_nebula.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_nebula.h"
#if LV_USE_LABEL != 0
#include "../core/lv_obj.h"
#include "../misc/lv_assert.h"
#include "../core/lv_group.h"
#include "../draw/lv_draw.h"
#include "../misc/lv_color.h"
#include "../misc/lv_math.h"
#include "../misc/lv_bidi.h"
#include "../misc/lv_txt_ap.h"
#include "../misc/lv_printf.h"
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

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_nebula_class

/**********************
 *      TYPEDEFS
 **********************/
 
/**********************
 *      DECLARATIONS
 **********************/
extern void gw_nebula_position_update(int16_t x_offset, int16_t y_offset,
                               const float x_init_array[], const float y_init_array[],
                               float x_array[], float y_array[],
                               float size_array[], uint16_t icon_num);
extern const uint32_t g_app_png_icon_addrs[];
extern hal_gfx_cmdlist_t * lv_port_get_current_cl(void);

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_nebula_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_nebula_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_nebula_event(const lv_obj_class_t * class_p, lv_event_t * e);
static void draw_main(lv_event_t * e);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_nebula_class = {
    .constructor_cb = lv_nebula_constructor,
    .destructor_cb = lv_nebula_destructor,
    .event_cb = lv_nebula_event,
    .instance_size = sizeof(lv_nebula_t),
    .base_class = &lv_obj_class
};

/**********************
 *      MACROS
 **********************/

/**********************
 *      STATIC VARIABLES
 **********************/
/* 9x9 Nebula Layout */
static const float x_init_array[CFG_ICON_CNT] = {
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
    -440.0,-330.0,-220.0,-110.0,0.0,110.0,220.0,330.0,440.0,
};

static const float y_init_array[CFG_ICON_CNT] = {
    -440.0,-440.0,-440.0,-440.0,-440.0,-440.0,-440.0,-440.0,-440.0,
    -330.0,-330.0,-330.0,-330.0,-330.0,-330.0,-330.0,-330.0,-330.0,
    -220.0,-220.0,-220.0,-220.0,-220.0,-220.0,-220.0,-220.0,-220.0,
    -110.0,-110.0,-110.0,-110.0,-110.0,-110.0,-110.0,-110.0,-110.0,
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
    110.0,110.0,110.0,110.0,110.0,110.0,110.0,110.0,110.0,
    220.0,220.0,220.0,220.0,220.0,220.0,220.0,220.0,220.0,
    330.0,330.0,330.0,330.0,330.0,330.0,330.0,330.0,330.0,
    440.0,440.0,440.0,440.0,440.0,440.0,440.0,440.0,440.0,
};


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_nebula_layout_update(lv_nebula_t* obj){
    gw_nebula_position_update(obj->offset.x, obj->offset.y, x_init_array, y_init_array,
                              obj->x_array, obj->y_array, obj->size_array, CFG_ICON_CNT);
}

lv_obj_t * lv_nebula_create(lv_obj_t * parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static lv_point_t _lastest_nebula_offset = {ICON_ALIGN_SIZE, ICON_ALIGN_SIZE};
static void lv_nebula_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    lv_nebula_t* nebula = (lv_nebula_t*)(obj);
    nebula->offset.x = _lastest_nebula_offset.x;
    nebula->offset.y = _lastest_nebula_offset.y;
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
}

static void lv_nebula_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
}

static void lv_nebula_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);
    if(lv_scr_act() != e->current_target) return; /* Stop if the object is deleted */
    lv_event_code_t code = lv_event_get_code(e);
    if(LV_EVENT_DRAW_POST == e->code){
       draw_main(e);
    }
}

static void draw_main(lv_event_t * e)
{
    lv_nebula_t* obj = (lv_nebula_t*)e->current_target;
    _lastest_nebula_offset.x = obj->offset.x;
    _lastest_nebula_offset.y = obj->offset.y;
    lv_nebula_layout_update((lv_nebula_t*)obj);
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t* cur_cl = lv_port_get_current_cl();
     // improve efficiency by remove the clean margin
    hal_gfx_set_clip(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, SCREEN_WIDTH, SCREEN_HEIGHT, HAL_GFX_RGB565, -1);
    hal_gfx_clear(0x0000);

    // Draw the app icons
    hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
    for (int16_t i = 0; i < CFG_ICON_CNT; i++){
        uint16_t draw_size = obj->size_array[i];
        if (0 == draw_size) continue;
        uint8_t icon_index = i % 36;
        uint32_t icon_src_addr = g_app_png_icon_addrs[icon_index] + QSPI0_XIP_BASE;
        hal_gfx_bind_src_tex(icon_src_addr, ICON_X_SIZE, ICON_Y_SIZE, HAL_GFX_RGB565, -1, HAL_GFX_FILTER_BL);
        hal_gfx_blit_rect_fit(obj->x_array[i], obj->y_array[i], draw_size, draw_size);
    }
    hal_gfx_cl_submit(cur_cl);
    hal_gfx_cl_wait(cur_cl);
}

#endif
