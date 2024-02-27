/**
 * @file lv_nebula.h
 *
 */

#ifndef LV_NEBULA_H
#define LV_NEBULA_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../lv_conf_internal.h"
#include <stdarg.h>
#include "../core/lv_obj.h"
#include "../draw/lv_draw.h"


/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    uint16_t x_size;
    uint16_t y_size;
    uint16_t item_num;
    uint32_t *p_icon_arrays;
    char *p_str_arrays;
    int16_t slide_offset;
}scale_list_attribute_t;

typedef struct {
    lv_obj_t obj;
    scale_list_attribute_t attribute;
} lv_scale_list_t;

#define SCREEN_WIDTH       (454)
#define SCREEN_HEIGHT      (454)

#define GW_MATH_ABS(x)     ((x) > 0 ? (x) : (-(x)))
#define SCALE_NORMAL_MODE            0                      //scale normal
#define SCALE_BACK_MODE              1                      //round rect scale normal
#define SCALE_CIRCLE_MODE            2                      //circle normal
#define SCALE_GRID_MODE              3
#define SCALE_NEBULA_MODE            4
typedef uint8_t scale_list_type_t;

extern const lv_obj_class_t lv_scalelist_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a nebula objects
 * @param parent    pointer to an object, it will be the parent of the new nebula.
 * @return          pointer to the created button
 */
lv_obj_t * lv_scale_list_create(lv_obj_t * parent);

/**
 * Update a nebula object
 * @param parent    pointer to the nebula object.
 */
void lv_scale_list_layout_update(lv_obj_t* obj);

/**
 * add item to scale list
 * @param obj              pointer to a scale list obj.
 * @param p_icon_arrays    pointer to img address array.
 * @param p_str_arrays     pointer to string array.
 * @param item_num         the number of item.
 */
void lv_scale_list_add_item(lv_obj_t* obj, uint32_t *p_icon_arrays, char *p_str_arrays, uint32_t item_num);

/**
 * get press item
 * @param obj              pointer to a scale list obj.
 * @param act_point        pointer to press point.
 * @param index            pointer to press item index.
 * @return                 ture means press item;false means no
**/

bool lv_scale_list_search_view(lv_obj_t* obj, lv_point_t act_point, uint8_t *index);

/**
 * set scale list mode
 * @param obj              pointer to a scale list obj.
 * @param type             setting different mode for scale list
**/

void lv_scale_list_set_mode(lv_obj_t* obj, scale_list_type_t type);

/**
 * set scale list mode
 * @param obj              pointer to a scale list obj.
 * @param type             get scalelist mode
**/

void lv_scale_list_get_mode(lv_obj_t* obj, scale_list_type_t* type);

/**
 * get scale list max offset
 * @param obj              pointer to a scale list obj.
 * @return                 the max offset of the list obj
**/

int16_t lv_scale_list_get_max_offset(lv_obj_t* obj);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_NEBULA_H*/
