/**
 * @file lv_nebula.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_scale_list.h"
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
#include "app_graphics_mem.h"
#include <math.h>
/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_scale_list_class

/**********************
 *      TYPEDEFS
 **********************/
 
/**********************
 *      DECLARATIONS
 **********************/
extern hal_gfx_cmdlist_t * lv_port_get_current_cl(void);

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_scale_list_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_scale_list_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_scale_list_event(const lv_obj_class_t * class_p, lv_event_t * e);
static void draw_main(lv_event_t * e);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_scale_list_class = {
    .constructor_cb = lv_scale_list_constructor,
    .destructor_cb = lv_scale_list_destructor,
    .event_cb = lv_scale_list_event,
    .instance_size = sizeof(lv_scale_list_t),
    .base_class = &lv_obj_class
};

/**********************
 *      MACROS
 **********************/

/**********************
 *      STATIC VARIABLES
 **********************/
#define DRAW_BACKGROUND     0


#define BACK_GROUND_MARGIN_X      42
#define NORMAL_MARGIN_X           82
#define MARGIN_Y                  40
#define SPACING                   18
#define ICON_Y_SIZE               80
#define ICON_X_SIZE               80
#define STR_X_SIZE                160
#define STR_Y_SIZE                50
#define CHAR_LENGTH               15


#define ANGLE_CHANGE    0.47f
#define CIRCLE_RADUIS   0.01745f
// Estimated by if the clock dial select view be overriten
#define SCALE_LIST_VIEW_SIZE     (5*(454*454*2))

typedef struct 
{
    uint8_t render_flag : 1;
    uint8_t scale_flag : 1;
    uint8_t end_flag : 1;
    uint8_t index;
    int16_t icon_start_x;
    int16_t icon_start_y;
    uint16_t scaled_x_size;
    uint16_t scaled_y_size;
    float scale;
    int16_t angle;
}scale_list_draw_info_t;


extern void gx_user_get_src_mode(uint8_t mode);

static uint16_t ITEM_X_OFFSET = 0;
static uint16_t MARGIN_X = 0;

static scale_list_type_t   show_mode = SCALE_NORMAL_MODE;
static int16_t s_scale_list_max_offset = 0;
static char *s_show_str;
static uint16_t ver_res;
static uint16_t hor_res;
static scale_list_attribute_t *attr;

static  scale_list_draw_info_t  draw_info[15];
static uint8_t render_count;
static int16_t slide_offset;
static void* s_scale_list_buf = NULL;
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t * lv_scale_list_create(lv_obj_t * parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_scale_list_t* scale_list = (lv_scale_list_t*)(obj);
    scale_list->attribute.slide_offset = slide_offset;
    //lv_scale_list_layout_update(scale_list);
    return obj;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_scale_list_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_scale_list_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
}

void lv_scale_list_layout_update(lv_obj_t* obj)
{
}
void gw_scale_list_create(const lv_scale_list_t *view);
void lv_scale_list_add_item(lv_obj_t* obj, uint32_t *p_icon_arrays, char *p_str_arrays, uint32_t item_num)
{
    lv_scale_list_t* scale_list = (lv_scale_list_t*)obj;
    scale_list->attribute.p_icon_arrays = p_icon_arrays;
    scale_list->attribute.p_str_arrays = p_str_arrays;
    scale_list->attribute.item_num = item_num;
    gw_scale_list_create(scale_list);
}

static void lv_scale_list_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);
    lv_res_t res;
    /*Call the ancestor's event handler*/
    res = lv_obj_event_base(MY_CLASS, e);
    if(res != LV_RES_OK) return;
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_DRAW_POST) { // LV_EVENT_DRAW_POST_END
        draw_main(e);
    }
}

void lv_scale_list_clear_offset(void)
{
    slide_offset = 0;
}

void lv_scale_list_set_mode(lv_obj_t* obj, scale_list_type_t type)
{
    show_mode = type;
    if(obj){
        slide_offset = 0;
        ((lv_scale_list_t* )obj)->attribute.slide_offset = 0;
        gw_scale_list_create((lv_scale_list_t* )obj);
    }
}

void lv_scale_list_get_mode(lv_obj_t* obj, scale_list_type_t* type)
{
    *type = show_mode;
}

int16_t lv_scale_list_get_max_offset(lv_obj_t* obj)
{
    return s_scale_list_max_offset;
}
//Check whether the current tp coordinates are in a click area
static bool gw_check_view(lv_point_t *act_point, lv_area_t *click_area)
{
    if( act_point->x  >= (click_area->x1) &&
            act_point->x  <=(click_area->x2) &&
            act_point->y  >= (click_area->y1) &&
            act_point->y  <=(click_area->y2))
    {
        return true;
    }
    return false;
}

bool lv_scale_list_search_view(lv_obj_t* obj, lv_point_t act_point, uint8_t *index)
{
    lv_area_t click_area;
    
    if(show_mode != SCALE_GRID_MODE)
    {
        for(uint8_t i=0; i<render_count; i++)
        {
            click_area.x1 = draw_info[i].icon_start_x;
            click_area.x2 =  click_area.x1 + (ICON_X_SIZE + SPACING + STR_X_SIZE) * draw_info[i].scale;
            click_area.y1 = draw_info[i].icon_start_y;
            click_area.y2 = click_area.y1 + ICON_Y_SIZE * draw_info[i].scale;
            if(gw_check_view(&act_point, &click_area))
            {
                *index = draw_info[i].index;
                return true;
            }
        }
    }
    else
    {
        for(uint8_t i=0; i<render_count; i++)
        {
            click_area.x1 = draw_info[i].icon_start_x;
            click_area.x2 =  click_area.x1 + (ICON_X_SIZE) * draw_info[i].scale;
            click_area.y1 = draw_info[i].icon_start_y;
            click_area.y2 = click_area.y1 + ICON_Y_SIZE * draw_info[i].scale;
            if(gw_check_view(&act_point, &click_area))
            {
                *index = draw_info[i].index;
                return true;
            }
        }
    }
    return false;
}

void gw_gpu_draw_progress(uint8_t progress, bool start)
{
    uint8_t  angle_step = 5;
    uint16_t ver_res = DISP_VER_RES;
    uint16_t hor_res = DISP_HOR_RES;
    lv_area_t clip_area = {0, 0, ver_res-1, hor_res - 1};
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    lv_point_t center = {227, 227};
    uint16_t radius = 210;
    // uint16_t width = 6;
    uint16_t start_angle = 330;
    uint16_t end_angle = 30;

    arc_dsc.color = lv_color_white();
    arc_dsc.width = 6;
    lv_draw_arc(center.x, center.y, radius, start_angle, end_angle, &clip_area, &arc_dsc);
    
    int16_t angle_offset = (progress * 55) / 100;
    
    if (start){
        start_angle = 330 - angle_offset;
        end_angle = start_angle + angle_step;
    }else{
        start_angle = 330 + angle_offset;
        end_angle = start_angle + angle_step;
    }
    arc_dsc.color.full = 0x0000FF80;
    lv_draw_arc(center.x, center.y, radius, start_angle, end_angle, &clip_area, &arc_dsc);
}

/*---------------------------------------------------------------------------------*/
static bool normal_scale_strategy(uint8_t item_index, uint8_t render_index, scale_list_draw_info_t *p_draw_info)
{
    float temp = 0.0;
    int16_t icon_start_y_temp;
    icon_start_y_temp = item_index *(ICON_Y_SIZE + SPACING) + attr->slide_offset + MARGIN_Y;
    int16_t icon_end_y = icon_start_y_temp + ICON_Y_SIZE;
    p_draw_info[render_index].render_flag = true;
    p_draw_info[render_index].icon_start_y = icon_start_y_temp;
    p_draw_info[render_index].icon_start_x = ITEM_X_OFFSET;
    
    if(icon_end_y < 0)
    {
        p_draw_info[render_index].render_flag = false;
    }
    else if(icon_start_y_temp < MARGIN_Y)
    {
        p_draw_info[render_index].end_flag = false;
        p_draw_info[render_index].scale_flag = true;
        temp = icon_end_y - MARGIN_Y;
        p_draw_info[render_index].scale = (float)(temp / (ICON_Y_SIZE));
    }
    else if(icon_start_y_temp >= (ver_res+MARGIN_Y))
    {
        p_draw_info[render_index].render_flag = false;
        return true;
    }
    else if(icon_end_y > (ver_res - MARGIN_Y))// need scale
    {
        p_draw_info[render_index].end_flag = true;
        p_draw_info[render_index].scale_flag = true;
        temp = ver_res - icon_start_y_temp - MARGIN_Y;
        p_draw_info[render_index].scale = (float)(temp / ICON_Y_SIZE);
    }
    else
    {
        p_draw_info[render_index].scale = 1.0;
    }
    return false;
}

/*---------------------------------------------------------------------------------*/
static bool circle_scale_strategy(uint8_t item_index, uint8_t render_index, scale_list_draw_info_t *p_draw_info)
{
    float angle_offset = (attr->slide_offset * ANGLE_CHANGE) + 45;
    int16_t icon_start_angle = item_index * 45 + (int16_t)(angle_offset) + 90;
    p_draw_info[render_index].angle = icon_start_angle;
     p_draw_info[render_index].render_flag = true;
    if(icon_start_angle < 90)
    {
        p_draw_info[render_index].render_flag = false;
    }
    else if(icon_start_angle > 270)
    {
        p_draw_info[render_index].render_flag = false;
        return true;
    }
    else if(icon_start_angle < 113)
    {
         p_draw_info[render_index].scale_flag = true;
         p_draw_info[render_index].scale = ((icon_start_angle - 90)/22.0) * 0.7;
    }
    else if(icon_start_angle < 158)
    {
         p_draw_info[render_index].scale_flag = true;
         p_draw_info[render_index].scale = ((icon_start_angle - 113)/45.0);
         if(p_draw_info[render_index].scale < 0.7f)
         {
             p_draw_info[render_index].scale = 0.7;
         }
    }
    else if(icon_start_angle < 203)
    {
        p_draw_info[render_index].scale_flag = false;
        p_draw_info[render_index].scale = 1.0;
    }
    else if(icon_start_angle < 248)
    {
        p_draw_info[render_index].scale_flag = true;
        p_draw_info[render_index].scale = ((248 - icon_start_angle)/45.0);
        if(p_draw_info[render_index].scale < 0.7f)
        {
            p_draw_info[render_index].scale = 0.7;
        }
    }
    else
    {
        p_draw_info[render_index].scale_flag = true;
        p_draw_info[render_index].scale = ((270.0 - icon_start_angle)/22.0) * 0.7;
    }
    
    return false;
}

/*---------------------------------------------------------------------------------*/
static void normal_cal_coord_strategy(uint8_t item_index, uint8_t render_index, scale_list_draw_info_t *p_draw_info)
{
    if(p_draw_info[render_index].scale_flag)
    {
        p_draw_info[render_index].icon_start_x = ITEM_X_OFFSET + ((ICON_X_SIZE - p_draw_info[render_index].scaled_x_size) >> 1);
        if(p_draw_info[render_index].end_flag == false)
        {
            int16_t icon_start_y_next = (item_index+1)*(ICON_Y_SIZE + SPACING) + attr->slide_offset;
            p_draw_info[render_index].icon_start_y = icon_start_y_next - p_draw_info[render_index].scaled_y_size - SPACING + MARGIN_Y;
        }
        else
        {
            int16_t icon_start_y_before = (item_index-1)*(ICON_Y_SIZE + SPACING) + attr->slide_offset;
            p_draw_info[render_index].icon_start_y = icon_start_y_before + ICON_Y_SIZE + SPACING  + MARGIN_Y;
        }
        if(show_mode == SCALE_BACK_MODE)
        {
            p_draw_info[render_index].icon_start_x +=  ((1.0f - p_draw_info[render_index].scale) * MARGIN_X);
        }
    }
}
/*---------------------------------------------------------------------------------*/
static void circle_cal_coord_strategy(uint8_t item_index, uint8_t render_index, scale_list_draw_info_t *p_draw_info)
{
    float anglef = p_draw_info[render_index].angle;
    float center_x = (165 * cos(anglef*CIRCLE_RADUIS));
    float center_y = (165 * sin(anglef*CIRCLE_RADUIS));
    int icon_center_x = center_x + 227;
    int icon_center_y = (int)center_y;
    if(icon_center_y >= 0)
    {
        icon_center_y = 227 - icon_center_y;
    }
    else
    {
        icon_center_y = -icon_center_y + 227;
    }
    p_draw_info[render_index].icon_start_x = icon_center_x - (p_draw_info[render_index].scaled_x_size >>1);
    p_draw_info[render_index].icon_start_y = icon_center_y - (p_draw_info[render_index].scaled_y_size >>1);
}

/*---------------------------------------------------------------------------------*/
static uint8_t get_draw_info(const lv_scale_list_t * view, scale_list_draw_info_t *p_draw_info)
{
    uint8_t index_count = 0;
    uint16_t item_num = attr->item_num;
    
    for(uint8_t i=0; i<item_num; i++)
    {
        bool render_finish = false;
        if(show_mode == SCALE_CIRCLE_MODE)
        {
            render_finish = circle_scale_strategy(i, index_count, p_draw_info);
        }
        else
        {
           render_finish = normal_scale_strategy(i, index_count, p_draw_info);
        }
        
        if(render_finish)
        {
            return index_count;
        }
        else
        {
            if(p_draw_info[index_count].render_flag)
            {
                p_draw_info[index_count].scaled_x_size = p_draw_info[index_count].scale * ICON_X_SIZE;
                p_draw_info[index_count].scaled_y_size = p_draw_info[index_count].scale * ICON_Y_SIZE;
                if(show_mode == SCALE_CIRCLE_MODE)
                {
                    circle_cal_coord_strategy(i, index_count, p_draw_info);
                }
                else
                {
                    normal_cal_coord_strategy(i, index_count, p_draw_info);
                }
                 p_draw_info[index_count].index = i;
                 index_count ++;
            }
        }
    }
    return index_count;
}

/*-------------------------draw round rect-------------------------------------------------------*/
static void gw_draw_scale_list_background(scale_list_draw_info_t *draw_info, uint8_t render_count)
{

    hal_gfx_set_clip(0, 0, hor_res, ver_res);
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    for(uint8_t i=0; i<render_count; i++)
    {
        uint8_t raduis = draw_info[i].scale * 20;
        uint8_t start_x = draw_info[i].icon_start_x - 20;
        uint16_t x_size = draw_info[i].scaled_y_size + SPACING + (draw_info[i].scale * STR_X_SIZE)  + 80;
        hal_gfx_fill_rounded_rect(start_x, draw_info[i].icon_start_y - 3, x_size, draw_info[i].scaled_y_size+6, raduis, 0x999d9c);
        
    }
}

/*-------------------------draw icon and str info--------------------------------------------*/
static void gw_draw_icon_and_str(scale_list_draw_info_t *draw_info, uint8_t render_count)
{
    hal_gfx_set_clip(0, 0, hor_res, ver_res);

    uint32_t cache_start_addr = (uint32_t)s_scale_list_buf;
    uint32_t cache_buf_addr = 0;
    uint32_t cache_addr_offset =  ver_res * STR_Y_SIZE * 2;
    uint32_t icon_src_addr = 0;
    hal_gfx_tex_format_t  text_format = HAL_GFX_RGB565;
    uint8_t index_offset = 36;
    if(show_mode == SCALE_BACK_MODE)
    {
        index_offset = 0;
        text_format = HAL_GFX_RGBA8888;
        //gx_user_get_src_mode(2);
    }
    
    for(uint8_t i=0; i<render_count; i++)
    {
        icon_src_addr = attr->p_icon_arrays[draw_info[i].index + index_offset] + QSPI0_XIP_BASE;
        hal_gfx_bind_src_tex(icon_src_addr, ICON_X_SIZE, ICON_Y_SIZE, text_format, -1, HAL_GFX_FILTER_BL);
        hal_gfx_blit_rect_fit(draw_info[i].icon_start_x, draw_info[i].icon_start_y,
                              draw_info[i].scaled_x_size, draw_info[i].scaled_y_size);
        
        uint16_t scaled_str_x_size = draw_info[i].scale * STR_X_SIZE;
        uint16_t scaled_str_y_size = draw_info[i].scale * STR_Y_SIZE;
        int16_t str_start_x = draw_info[i].icon_start_x + draw_info[i].scaled_x_size + SPACING;
        int16_t str_start_y = ((draw_info[i].scaled_y_size - scaled_str_y_size)>>1) + draw_info[i].icon_start_y;
  
        cache_buf_addr = cache_start_addr + (cache_addr_offset * draw_info[i].index);
        hal_gfx_bind_src_tex(cache_buf_addr, STR_X_SIZE, STR_Y_SIZE, HAL_GFX_RGB565, hor_res*2, HAL_GFX_FILTER_BL);
        
        hal_gfx_blit_rect_fit(str_start_x, str_start_y, scaled_str_x_size, scaled_str_y_size);
    }
}

#define MARGIN_GRID             82
#define SPACING_GRID            25
static void gw_draw_grid_list(uint16_t draw_y_start, const lv_scale_list_t * obj)
{
    uint8_t item_count = attr->item_num / 3;
    uint8_t icon_remain = attr->item_num % 3;
    if(icon_remain !=0)
    {
        item_count += 1;
    }
    uint8_t one_item_icon_count = 0;
    int16_t icon_start_y = 0;
    float scale = 1.0;
    bool render_flag = false;
    bool scale_flag = false;
    bool end_flag = false;
    

    render_count = 0;
    
    for(uint16_t i=0; i<item_count; i++)
    {
        /*----------------------cale scale info-----------------------------*/
        icon_start_y = i*(ICON_Y_SIZE + SPACING_GRID) + attr->slide_offset + MARGIN_GRID;
        
        scale = 1.0;
        render_flag = true;
        scale_flag = false;
        float temp = 0.0;
        int16_t icon_end_y = icon_start_y + ICON_Y_SIZE;
        
        if(icon_end_y < 0)
        {
            render_flag = false;
        }
        else if(icon_start_y < MARGIN_GRID)
        {
            scale_flag = true;
            temp = icon_start_y;
            scale = (float)(temp / (ICON_Y_SIZE));
            end_flag = false;
        }
        else if(icon_start_y >= ver_res)
        {
            render_flag = false;
            break;
        }
        else if(icon_end_y > (ver_res - MARGIN_GRID))// need scale
        {
            scale_flag = true;
            temp = ver_res - icon_end_y;
            scale = (float)(temp / ICON_Y_SIZE);
            end_flag = true;
        }
        
        /*----------------------------render---------------------------------*/        
        if(render_flag)
        {
            if(i == item_count -1)
            {
                if(icon_remain == 0)
                {
                    one_item_icon_count = 3;
                }
                else
                {
                    one_item_icon_count = icon_remain;
                }
            }
            else
            {
                one_item_icon_count = 3;
            }
           
            uint32_t icon_src_addr = 0;
            uint16_t scaled_icon_x_size = scale * ICON_X_SIZE;
            uint16_t scaled_icon_y_size = scale * ICON_Y_SIZE;
            uint16_t icon_start_x = MARGIN_GRID;
            
            if(scale_flag)
            {
                icon_start_x = MARGIN_GRID + ((ICON_X_SIZE - scaled_icon_x_size) >> 1);
                if(end_flag == false)
                {
                    int16_t icon_start_y_next = (i+1)*(ICON_Y_SIZE + SPACING_GRID) + attr->slide_offset;
                    icon_start_y = icon_start_y_next - scaled_icon_y_size - SPACING_GRID + MARGIN_GRID;
                }
                else
                {
                    int16_t icon_start_y_before = (i-1)*(ICON_Y_SIZE + SPACING_GRID) + attr->slide_offset;
                    icon_start_y = icon_start_y_before + ICON_Y_SIZE + SPACING_GRID  + MARGIN_GRID;
                }
            }
  
            for(uint16_t j=0; j<one_item_icon_count; j++)
            {
                icon_src_addr = attr->p_icon_arrays[i*3+j+36] + QSPI0_XIP_BASE;
                
                hal_gfx_bind_src_tex(icon_src_addr, ICON_X_SIZE, ICON_Y_SIZE, HAL_GFX_RGB565, -1, HAL_GFX_FILTER_BL);
                hal_gfx_blit_rect_fit(icon_start_x, icon_start_y, scaled_icon_x_size, scaled_icon_y_size);
                draw_info[render_count].index = i*3+j;
                draw_info[render_count].icon_start_x = icon_start_x;
                draw_info[render_count].icon_start_y = icon_start_y;
                draw_info[render_count].scale = scale;
                render_count++;
                icon_start_x += (SPACING_GRID + ICON_X_SIZE);
            }
        }
    }
}

/*---------------------------------------------------------------------------------*/

static void draw_main(lv_event_t * e)
{
    lv_scale_list_t* obj = (lv_scale_list_t*)e->current_target;
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    hal_gfx_cmdlist_t* cur_cl = lv_port_get_current_cl();
    slide_offset = obj->attribute.slide_offset;

    // Draw the images
    // hal_gfx_cl_bind_circular(cur_cl);
    hal_gfx_bind_dst_tex((uint32_t)draw_buf->buf_act, SCREEN_WIDTH, SCREEN_HEIGHT, HAL_GFX_RGB565, -1);
    hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
    
    if(show_mode == SCALE_GRID_MODE)
    {
        gw_draw_grid_list(0, obj);
    }
    else
    {
        render_count = get_draw_info(obj, draw_info);
         
        if(show_mode == SCALE_BACK_MODE) // draw round background
        {
            gw_draw_scale_list_background(draw_info, render_count);
        }
        gw_draw_icon_and_str(draw_info, render_count);
    }

    // Destroy the GPU environment
    hal_gfx_cl_submit(cur_cl);
    hal_gfx_cl_wait(cur_cl);

    bool start_flag = false;
    if (attr->slide_offset > 0) start_flag = true;
    uint8_t pre = (GW_MATH_ABS(attr->slide_offset) * 100) / GW_MATH_ABS(s_scale_list_max_offset);
    gw_gpu_draw_progress(pre, start_flag);
}



void gw_scale_list_create(const lv_scale_list_t *view)
{
    if(NULL == s_scale_list_buf)
    {
        s_scale_list_buf = app_graphics_mem_malloc(SCALE_LIST_VIEW_SIZE);
        // printf("list create: %08X\n", (uint32_t)s_scale_list_buf);
    }
    ver_res = SCREEN_WIDTH;
    hor_res = SCREEN_HEIGHT;
    attr = (scale_list_attribute_t*)&view->attribute;// get attribute

    uint8_t item_count = 0;
    if(show_mode == SCALE_GRID_MODE)
    {
        item_count = view->attribute.item_num / 3;
        uint8_t icon_remain = view->attribute.item_num % 3;
        if(icon_remain !=0)
        {
            item_count += 1;
        }
        s_scale_list_max_offset = - (item_count - 2) * (ICON_Y_SIZE + SPACING);
    }
    else
    {
        lv_disp_t* disp = _lv_refr_get_disp_refreshing();
        lv_disp_draw_buf_t* origin_draw_buf = disp->driver->draw_buf;

        static lv_disp_draw_buf_t pre_render_draw_buf;
        lv_disp_draw_buf_init(&pre_render_draw_buf, s_scale_list_buf, NULL, DISP_HOR_RES * DISP_VER_RES);
        disp->driver->draw_buf = &pre_render_draw_buf;
        lv_area_t mask = {0, 0, 453, 453};
        memcpy(&pre_render_draw_buf.area, &mask, sizeof(mask));
        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        extern lv_font_t lv_font_msyhbd_40;
        label_dsc.font = &lv_font_msyhbd_40;
        label_dsc.color = lv_color_white();
        if(show_mode == SCALE_BACK_MODE)
        {
            MARGIN_X = BACK_GROUND_MARGIN_X;
        }
        else
        {
            MARGIN_X = NORMAL_MARGIN_X;
        }
        
        ITEM_X_OFFSET = MARGIN_X + 30;

       
        item_count = view->attribute.item_num;
        scale_list_attribute_t *attr = (scale_list_attribute_t*)&view->attribute;// get attribute
        
        s_scale_list_max_offset = - (item_count - 3) * (ICON_Y_SIZE + SPACING) + 40;
 
        uint32_t cache_buf_addr = (uint32_t)s_scale_list_buf;
        memset((void *)cache_buf_addr, 0x00, SCALE_LIST_VIEW_SIZE);
        hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
        hal_gfx_cmdlist_t* cl = &cmd;
        hal_gfx_cl_bind_circular(cl);
        lv_area_t area = {0, 0, 159, 49};
        // gw_color_fill_by_gpu(cache_buf_addr, init_color, ver_res, STR_Y_SIZE *2 * item_count);
        for(uint16_t i=0; i<item_count; i++)
        {
            //gw_set_buf_act_to_set(cache_buf_addr);
            lv_disp_draw_buf_init(&pre_render_draw_buf, (void *)cache_buf_addr, NULL, DISP_HOR_RES * DISP_VER_RES);
            memcpy(&pre_render_draw_buf.area, &mask, sizeof(mask));
            disp->driver->draw_buf = &pre_render_draw_buf;
            s_show_str = (char*)(attr->p_str_arrays + (CHAR_LENGTH*i));
           // gw_draw_str_label(0, (const gw_view_t *)&s_item_str_label);
            lv_draw_label(&area, &area, &label_dsc, s_show_str, NULL);
            cache_buf_addr += (ver_res * STR_Y_SIZE *2);
        }
        disp->driver->draw_buf = origin_draw_buf;
        hal_gfx_cl_destroy(cl);
    }
}


void gw_scale_list_destroy(const lv_scale_list_t *view)
{
    if(NULL != s_scale_list_buf)
    {
        //printf("list destroy: %08X\n", (uint32_t)s_scale_list_buf);
        app_graphics_mem_free(s_scale_list_buf);
        s_scale_list_buf = NULL;
    }
}

