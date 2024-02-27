/**
 * @file lv_cusom_obj.h
 *
 */

#ifndef LV_GRAPHICS_DRAW_H
#define LV_GRAPHICS_DRAW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lv_obj.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef enum {
    GRAPHICS_TYPE_CIRCLE,
    GRAPHICS_TYPE_ARCS,
    GRAPHICS_TYPE_STENCIL,
    GRAPHICS_TYPE_ROTATE_CUBE,
    GRAPHICS_TYPE_JUMPING_SHAPE,
    GRAPHICS_TYPE_TSC4_COMP,
} free_graphics_type_e;

/**
 * Create a base object (a rectangle)
 * @param parent    pointer to a parent object. If NULL then a screen will be created.
 * @return          pointer to the new object
 */
lv_obj_t * lv_graphics_draw_obj_create(lv_obj_t * parent);


/**
 * set clip area by user
 * @param clip_area   set new clip area
 */
void lv_set_graphics_draw_type(free_graphics_type_e type);



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_OBJ_H*/
