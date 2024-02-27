/**
 * @file lv_obj.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_obj.h"
#include "lv_indev.h"
#include "lv_refr.h"
#include "lv_group.h"
#include "lv_disp.h"
#include "lv_theme.h"
#include "../misc/lv_assert.h"
#include "../draw/lv_draw.h"
#include "../misc/lv_anim.h"
#include "../misc/lv_timer.h"
#include "../misc/lv_async.h"
#include "../misc/lv_fs.h"
#include "../misc/lv_gc.h"
#include "../misc/lv_math.h"
#include "../misc/lv_log.h"
#include "../hal/lv_hal.h"
#include "../extra/lv_extra.h"
#include <stdint.h>
#include <string.h>


/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_custom_obj_class


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_obj_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_obj_draw(lv_event_t * e);
static void lv_obj_event(const lv_obj_class_t * class_p, lv_event_t * e);
static lv_area_t new_clip_area = {0};


/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_custom_obj_class = {
    .constructor_cb = lv_obj_constructor,
    .event_cb = lv_obj_event,
    .instance_size = (sizeof(lv_obj_t)),
    .base_class = &lv_obj_class,
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/


lv_obj_t * lv_custom_obj_create(lv_obj_t * parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_custom_obj_set_clip_area(lv_area_t clip_area)
{
    new_clip_area = clip_area;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_obj_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_t * parent = obj->parent;
    obj->flags = LV_OBJ_FLAG_CLICKABLE;
    obj->flags |= LV_OBJ_FLAG_SNAPPABLE;
    obj->flags |= LV_OBJ_FLAG_SCROLLABLE;
    if(parent) obj->flags |= LV_OBJ_FLAG_PRESS_LOCK;
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_obj_draw(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    
    if(code == LV_EVENT_DRAW_MAIN) {
        const lv_area_t * clip_area = lv_event_get_param(e);
        lv_draw_rect_dsc_t draw_dsc;
        lv_draw_rect_dsc_init(&draw_dsc);
        /*If the border is drawn later disable loading its properties*/
        if(lv_obj_get_style_border_post(obj, LV_PART_MAIN)) {
            draw_dsc.border_post = 1;
        }

        lv_obj_init_draw_rect_dsc(obj, LV_PART_MAIN, &draw_dsc);

        lv_coord_t w = lv_obj_get_style_transform_width(obj, LV_PART_MAIN);
        lv_coord_t h = lv_obj_get_style_transform_height(obj, LV_PART_MAIN);
        lv_area_t coords;
        lv_area_copy(&coords, &obj->coords);
        coords.x1 -= w;
        coords.x2 += w;
        coords.y1 -= h;
        coords.y2 += h;

        lv_obj_draw_part_dsc_t part_dsc;
        lv_obj_draw_dsc_init(&part_dsc, clip_area);
        part_dsc.class_p = MY_CLASS;
        part_dsc.type = LV_OBJ_DRAW_PART_RECTANGLE;
        part_dsc.rect_dsc = &draw_dsc;
        part_dsc.draw_area = &coords;
        part_dsc.part = LV_PART_MAIN;
        lv_event_send(obj, LV_EVENT_DRAW_PART_BEGIN, &part_dsc);
        if(new_clip_area.x1 != 0 && new_clip_area.x2 != 0){
            lv_draw_rect(&coords, &new_clip_area, &draw_dsc);
        }else{
            lv_draw_rect(&coords, clip_area, &draw_dsc);
        }

        lv_event_send(obj, LV_EVENT_DRAW_PART_END, &part_dsc);

    }
}


static void lv_obj_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_DRAW_MAIN || code == LV_EVENT_DRAW_POST || code == LV_EVENT_COVER_CHECK) {
        lv_obj_draw(e);
    }
}

