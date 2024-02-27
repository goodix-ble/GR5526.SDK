#include "string.h"
#include "dspi_screen_390.h"
#include "app_dspi.h"
#include "app_dspi_dma.h"
#include "image/hehua_454_rgba565.h"
#include "image/tiger_454_rgba565.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * Display Device Setting
 */
#define D_DISPLAY_PIXEL_WIDTH                       (454u)              /* screen width */
#define D_DISPLAY_PIXEL_HEIGHT                      (454u)              /* screen height */
#define D_DISPLAY_PIXEL_DEPTH                       (2u)
#define D_DISPLAY_TEST_SIZE                         (D_DISPLAY_PIXEL_WIDTH*D_DISPLAY_PIXEL_HEIGHT*D_DISPLAY_PIXEL_DEPTH)

#define D_DISPLAY_MODE                              D_DISPLAT_4W1L

static uint8_t  dspi_dev_init_display(void);

void dspi_display_refresh(void) 
{
    static bool display_flag = 0;
    if(0 == display_flag)
    {
        display_lcd((uint8_t *)&tiger_454_rgba565[0], D_DISPLAY_TEST_SIZE, D_DISPLAY_MODE);
        display_flag = 1;
    }
    else
    {
        display_lcd((uint8_t *)&hehua_454_rgba565[0], D_DISPLAY_TEST_SIZE, D_DISPLAY_MODE);
        display_flag = 0;
    }
    return;
}

void dspi_dev_application_task(void *p_arg) 
{
    dspi_dev_init_display();

    while(1) {
        dspi_display_refresh();
        printf(".");
        vTaskDelay(1000);
    }
}

static uint8_t dspi_dev_init_display(void) 
{
    dspi_screen_init_basic(D_DISPLAY_MODE);
    printf(">>> Q.Display Init OK!\r\n");
    return 0;
}
