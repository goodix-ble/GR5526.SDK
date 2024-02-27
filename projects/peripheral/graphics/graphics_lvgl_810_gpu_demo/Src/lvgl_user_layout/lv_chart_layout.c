/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "grx_hal.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "lv_layout_manager.h"
#include "app_graphics_mem.h"
#include "lv_wms.h"
#include "lv_port_disp.h"
#include "lv_gpu_draw_polyline.h"

/*
 * EXTERNAL DECLARATIONS
 *****************************************************************************************
 */

/*
 * DEFINE
 *****************************************************************************************
 */
#include "lvgl.h"
#include "lv_gpu_draw_polyline.h"

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define HERAT_POINT_COUNT    20

static vec_t vector[HERAT_POINT_COUNT] = {0};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static uint8_t heart_rate[30] = {43,53,56,58,70,
                                85,80,80,75,90,
                                80,75,82,77,66,
                                56,50,40,30,20,
                                67,67,64,64,64,
                                60,59,55,45,35};

static lv_obj_t * g_chart = NULL;
static lv_obj_t * g_chart_view = NULL;
static lv_chart_series_t * ser1;

static uint8_t line_color_index[HERAT_POINT_COUNT] = \
{
   1,1,//lv_palette_darken(LV_PALETTE_RED, 1),lv_palette_darken(LV_PALETTE_RED, 1),
   2,2,//lv_palette_darken(LV_PALETTE_RED, 2),lv_palette_darken(LV_PALETTE_RED, 2),
   3,3,//lv_palette_darken(LV_PALETTE_RED, 3),lv_palette_darken(LV_PALETTE_RED, 2),
   4,4,//lv_palette_darken(LV_PALETTE_RED, 3),lv_palette_darken(LV_PALETTE_RED, 3),
   3,2,//lv_palette_darken(LV_PALETTE_RED, 4),lv_palette_darken(LV_PALETTE_RED, 4), \\5
   1,1,//lv_palette_darken(LV_PALETTE_RED, 5),lv_palette_darken(LV_PALETTE_RED, 5), \\6
   1,2,//lv_palette_lighten(LV_PALETTE_RED, 1),lv_palette_lighten(LV_PALETTE_RED, 2),
   3,3,//lv_palette_lighten(LV_PALETTE_RED, 3),lv_palette_lighten(LV_PALETTE_RED, 3),
   4,4,//lv_palette_lighten(LV_PALETTE_RED, 4),lv_palette_lighten(LV_PALETTE_RED, 4),
   3,5,//lv_palette_lighten(LV_PALETTE_RED, 5),lv_palette_lighten(LV_PALETTE_RED, 5),
};


static void draw_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);

    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if(dsc->part == LV_PART_ITEMS) {
        if(!dsc->p1 || !dsc->p2) return;

        if (dsc->id == 0) {
            /*Draw a rectangle background*/
           lv_area_t bg_area;
           lv_draw_rect_dsc_t draw_bg_dsc;
           lv_draw_rect_dsc_init(&draw_bg_dsc);
           lv_coord_t border_width = lv_obj_get_style_border_width(obj, LV_PART_MAIN);
           lv_coord_t pad_left = lv_obj_get_style_pad_left(obj, LV_PART_MAIN) + border_width;
           bg_area.x1 = dsc->p1->x;
           bg_area.x2 = obj->coords.x2;
           bg_area.y1 = obj->coords.y1;
           bg_area.y2 = obj->coords.y2;

           draw_bg_dsc.bg_color = lv_palette_darken(LV_PALETTE_RED, 1);
           draw_bg_dsc.bg_grad_color = lv_color_black();//lv_palette_lighten(LV_PALETTE_BLUE, 4);
           draw_bg_dsc.bg_grad_dir = LV_GRAD_DIR_VER;

           lv_draw_rect(&bg_area, dsc->clip_area, &draw_bg_dsc);
        }

        vector[dsc->id].x = (double)dsc->p1->x;
        vector[dsc->id].y = (double)dsc->p1->y;

        lv_draw_rect_dsc_t draw_rect_dsc;
        lv_draw_rect_dsc_init(&draw_rect_dsc);
        draw_rect_dsc.bg_color = lv_color_black();

        lv_area_t a;

        lv_point_t tri_point[4] = {0};
        tri_point[0].x = dsc->p1->x;
        tri_point[0].y = obj->coords.y1;
        tri_point[1].x = dsc->p1->x;
        tri_point[1].y = dsc->p1->y + 20;
        if (dsc->id != 18)
            tri_point[3].x = tri_point[2].x = dsc->p2->x;
        else
            tri_point[3].x = tri_point[2].x = obj->coords.x2 + 1;

        tri_point[2].y = dsc->p2->y + 20;
//        tri_point[3].x = dsc->p2->x;
        tri_point[3].y = obj->coords.y1;
        lv_draw_polygon(tri_point, 4, dsc->clip_area, &draw_rect_dsc);

        if(dsc->id == 18)
        {
            a.x1 = dsc->p2->x - 6;
            a.x2 = dsc->p2->x + 6;
            a.y1 = dsc->p2->y - 6;
            a.y2 = dsc->p2->y + 6;
            draw_rect_dsc.radius = 6;
            draw_rect_dsc.bg_color = lv_palette_lighten(LV_PALETTE_RED, line_color_index[dsc->id]);
            lv_draw_rect(&a, dsc->clip_area, &draw_rect_dsc);

            /* Draw Polyline */
            vector[dsc->id + 1].x = (double)dsc->p2->x - 10;
            vector[dsc->id + 1].y = (double)dsc->p2->y;
            lv_draw_polyline(vector, 20, 0xff888888);

            lv_area_t offset_area = {
                .x1 = draw_buf->area.x1 + disp->driver->offset_x,
                .y1 = draw_buf->area.y1 + disp->driver->offset_y,
                .x2 = draw_buf->area.x2 + disp->driver->offset_x,
                .y2 = draw_buf->area.y2 + disp->driver->offset_y
            };
            disp->driver->flush_cb(disp->driver, &offset_area, NULL);
        }
        dsc->line_dsc->opa = LV_OPA_TRANSP;
    }

}

void lv_chart_layout_destroy(void)
{
    if(NULL == g_chart_view) return;
    lv_wms_deinit(g_chart_view);
    lv_obj_del(g_chart_view);
    g_chart_view = NULL;
}

lv_obj_t* lv_chart_layout_create(void)
{
    if(NULL != g_chart_view) return g_chart_view;
    g_chart_view = lv_obj_create(NULL);
    lv_obj_set_scrollbar_mode(g_chart_view, LV_SCROLLBAR_MODE_OFF);

    /*Create a g_chart*/
    g_chart = lv_chart_create(g_chart_view);
    lv_obj_set_style_bg_color(g_chart, lv_color_black(), 0);
    lv_obj_set_style_border_width(g_chart, 0, 0);

    lv_obj_set_size(g_chart, 360, 200);
    lv_obj_center(g_chart);
    lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/

    lv_chart_set_div_line_count(g_chart, 0, 0);

    lv_obj_add_event_cb(g_chart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_chart_set_update_mode(g_chart, LV_CHART_UPDATE_MODE_CIRCULAR);

    /*Add two data series*/
    ser1 = lv_chart_add_series(g_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_X, 0, 50);
    lv_chart_set_point_count(g_chart, 20);
    lv_obj_set_style_size(g_chart, 0, LV_PART_INDICATOR);
    //lv_chart_set_axis_tick(g_chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 5, 1, true , 40);
    uint32_t i;
    for(i = 0; i < HERAT_POINT_COUNT; i++) {
        lv_chart_set_next_value(g_chart, ser1, heart_rate[i]);
    }

    lv_wms_init(g_chart_view);
    lv_wms_self_destroy_func_set(g_chart_view, lv_chart_layout_destroy);
    lv_wms_left_create_func_set(g_chart_view, lv_activity_layout_create);
    lv_wms_right_create_func_set(g_chart_view, lv_heartrate_layout_create);
    lv_scr_load(g_chart_view);
    lvgl_mem_used_dump(__func__, __LINE__);
    return g_chart_view;
}
