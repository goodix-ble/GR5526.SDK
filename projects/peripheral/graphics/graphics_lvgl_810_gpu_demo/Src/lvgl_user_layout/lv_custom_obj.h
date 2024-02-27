/**
 * @file lv_cusom_obj.h
 *
 */

#ifndef LV_CUSTOM_OBJ_H
#define LV_CUSTOM_OBJ_H

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

/**
 * Create a base object (a rectangle)
 * @param parent    pointer to a parent object. If NULL then a screen will be created.
 * @return          pointer to the new object
 */
lv_obj_t * lv_custom_obj_create(lv_obj_t * parent);


/**
 * set clip area by user
 * @param clip_area   set new clip area
 */
void lv_custom_obj_set_clip_area(lv_area_t clip_area);



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_OBJ_H*/
