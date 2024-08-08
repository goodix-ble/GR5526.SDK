/**
 * @file lv_port_anim.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_disp.h"
#include "../misc/lv_math.h"
#include "../core/lv_refr.h"

#include "lv_wms_surface_flinger.h"
#include "lv_enhanced_anim.h"
#include "drv_adapter_display.h"
#include "app_graphics_dc.h"
#include "gr55xx_delay.h"


/*************************************************************************************************
 *                                   Static Declaration
 *************************************************************************************************/
static void _lv_port_scr_anim_transform_finish(lv_anim_t * a);
static void _lv_port_scr_anim_transform(void * obj, int32_t v);
static void _lv_port_scr_anim_frame_update(uint32_t degree);
static void _lv_port_scr_anim_cache(uint32_t );

 bool s_rotate_anim_transit_is_progress  = false;

/*************************************************************************************************
 *                                   PUBLIC Methods
 *************************************************************************************************/

lv_img_dsc_t *lv_snapshot_take_ext(lv_obj_t *obj, lv_img_cf_t cf)
{
    LV_ASSERT_NULL(obj);
    uint32_t buff_size = lv_snapshot_buf_size_needed(obj, cf);

    extern void * p_rgb_fb1;
    void *buf = p_rgb_fb1;//app_graphics_mem_malloc(buff_size);
    LV_ASSERT_MALLOC(buf);
    if(buf == NULL) {
        return NULL;
    }

    lv_img_dsc_t * dsc = lv_mem_alloc(sizeof(lv_img_dsc_t));
    LV_ASSERT_MALLOC(buf);
    if(dsc == NULL) {
        lv_mem_free(buf);
        return NULL;
    }

    if(lv_snapshot_take_to_buf(obj, cf, dsc, buf, buff_size) == LV_RES_INV) {
        lv_mem_free(buf);
        lv_mem_free(dsc);
        return NULL;
    }

    return dsc;
}

void lv_snapshot_free_ext(lv_img_dsc_t * dsc)
{
    if(!dsc)
        return;

    lv_mem_free(dsc);
}

void lv_scr_load_anim_rotate(lv_obj_t * cur, uint32_t src, uint32_t time, uint32_t delay, uint32_t start, uint32_t end, uint32_t anim, void * usr_data)
{
    int32_t (*p_anim_path_func[5])(const lv_anim_t * a) = {
        lv_anim_path_ease_in,
        lv_anim_path_ease_out,
        lv_anim_path_ease_in_out,
        lv_anim_path_overshoot,
        lv_anim_path_bounce
    };

    /* if transit anim is progress, postpone to load new screent in direct-load mode */
    if(s_rotate_anim_transit_is_progress) {
        return;
    }

    lv_wms_refresh_enabled_set(false);
    lv_wms_display_enabled_set(false);

    _lv_port_scr_anim_cache(src);

    s_rotate_anim_transit_is_progress = true;

    lv_anim_t a_new;
    lv_anim_init(&a_new);
    lv_anim_set_var(&a_new, cur);
    lv_anim_set_user_data(&a_new,  (void*)usr_data);
    lv_anim_set_values(&a_new, start, end);
    lv_anim_set_exec_cb(&a_new,    _lv_port_scr_anim_transform );
    lv_anim_set_ready_cb(&a_new,   _lv_port_scr_anim_transform_finish);
    lv_anim_set_time(&a_new,  time);
    lv_anim_set_delay(&a_new, delay);
    lv_anim_set_path_cb(&a_new, p_anim_path_func[anim%5]);

    lv_anim_start(&a_new);

    return;
}


/*************************************************************************************************
 *                                Static Implement
 *************************************************************************************************/

static void _lv_port_scr_anim_transform_finish(lv_anim_t * a)
{
    

    lv_wms_refresh_enabled_set(true);
    lv_wms_display_enabled_set(true);

    void * usr_data = lv_anim_get_user_data(a);

    lv_snapshot_free_ext(usr_data);
    lv_anim_set_user_data(a, (void*)NULL);
    
    s_rotate_anim_transit_is_progress = false;

    return;
}


static void _lv_port_scr_anim_frame_update(uint32_t degree) {

    lv_disp_t * disp                 = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf    = lv_disp_get_draw_buf(disp);

    // Render the transit view with slide offset
    lv_transit_frame_t * p_tf = lv_wms_transit_frame_rotate(draw_buf->buf_act, degree);
    // Flush the frame buffer to screen
    drv_adapter_disp_wait_to_flush();
    drv_adapter_disp_set_show_area(0, 0, p_tf->transit_scrn_res_w - 1, p_tf->transit_scrn_res_h - 1);
    drv_adapter_disp_flush((void *) draw_buf->buf_act, GDC_DATA_FORMAT_TSC4, p_tf->transit_scrn_res_w, p_tf->transit_scrn_res_h);

    return;
}

static void _lv_port_scr_anim_transform(void * obj, int32_t v){

    _lv_port_scr_anim_frame_update(v/10);

    return;
}


static void _lv_port_scr_anim_cache(uint32_t src_addr) {

    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_CURRENT, (uint32_t)src_addr);
    return;
}

