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

/*********************
 *      DEFINES
 *********************/
/* 9x9 Nebula Layout */
#define ITEM_COL_CNT      (9)
#define ITEM_ROW_CNT      (9)
#define ICON_SPACE_SIZE   (110)
#define ICON_ALIGN_SIZE   (55)
#define MIN_OFFSET        (-495)
#define MAX_OFFSET        (495)
#define ITEM_SIZE         (110)
#define ICON_Y_SIZE       (80)
#define ICON_X_SIZE       (80)
#define SCREEN_WIDTH      (454)
#define SCREEN_HEIGHT     (454)
#define SCREEN_RADIUS     (227.0f)
#define MAX_ICON_RADIUS   (48.0f)
#define CENTER_X          (454 >> 1)
#define CENTER_Y          (454 >> 1)
#define CFG_ICON_CNT      (ITEM_ROW_CNT * ITEM_COL_CNT)
#define MAX(a, b)         (((a) > (b)) ? (a) : (b))
#define MIN(a, b)         (((a) < (b)) ? (a) : (b))

/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
    lv_obj_t obj;
    lv_point_t offset;
    float x_array[CFG_ICON_CNT];
    float y_array[CFG_ICON_CNT];
    float size_array[CFG_ICON_CNT];
} lv_nebula_t;

extern const lv_obj_class_t lv_nebula_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a nebula objects
 * @param parent    pointer to an object, it will be the parent of the new nebula.
 * @return          pointer to the created button
 */
lv_obj_t * lv_nebula_create(lv_obj_t * parent);

/**
 * Update a nebula object
 * @param parent    pointer to the nebula object.
 */
void lv_nebula_layout_update(lv_nebula_t* obj);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_NEBULA_H*/
