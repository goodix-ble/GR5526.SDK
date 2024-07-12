#include "gr_includes.h"
#include "user_periph_setup.h"
#include "custom_config.h"
#include "board_SK.h"
#include "user_periph_setup.h"
#include "drv_adapter_norflash.h"
#include "drv_adapter_port.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "tusb.h"
#include "ff.h"

#include <stdio.h>
#include <stdbool.h>

#define USBD_STACK_SIZE (3 * configMINIMAL_STACK_SIZE / 2) * (CFG_TUSB_DEBUG ? 2 : 1)

enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t s_blink_interval_ms = BLINK_NOT_MOUNTED;

static void led_blinking_task(void *param)
{
    (void)param;

    while(1)
    {
        vTaskDelay(s_blink_interval_ms);
        app_io_toggle_pin(APP_IO_TYPE_MSIO, APP_LED_NUM_0_IO);
    }
}

static void usb_device_task(void* param)
{
    (void)param;

    drv_adapter_norflash_register();
    drv_adapter_norflash_init();

    // Try mount fatfs
    FATFS fs;
    FRESULT ret = f_mount(&fs, "", 1);
    if (ret != FR_OK)
    {
        // mount failed, do format
        printf("Mount failed, formatting...\n");
        MKFS_PARM parm = {
            .fmt = FM_FAT,
            .n_fat = 1,
            .align = 0,
            .n_root = 0,
            .au_size = 0,
        };

        ret = f_mkfs("", &parm, NULL, 4 * 1024);
        if (ret == FR_OK)
        {
            ret = f_mount(&fs, "", 1);
        }
        else
        {
            printf("[ASSERT] Format NOR FLASH failed: %d!\n", ret);
            while(1);
        }
    }

    if (ret != FR_OK)
    {
        printf("[ASSERT] Failed to mount FatFs: %d\n", ret);
        while(1);
    }

    printf("FatFs Mounted\n");

    tusb_init();

    while(1)
    {
        tud_task();
    }
}

void main(void)
{
    app_periph_init();
    SetSerialClock(SERIAL_N96M_CLK);

    printf("******************************************************\r\n");
    printf("*    USB device CDC class example.                   *\r\n");
    printf("******************************************************\r\n");

    xTaskCreate(led_blinking_task, "blinky", configMINIMAL_STACK_SIZE, NULL, 1,                        NULL);
    xTaskCreate(usb_device_task,   "usbd",   6*1024,          NULL, configMAX_PRIORITIES - 1, NULL);

    vTaskStartScheduler();
}

// TinyUSB Callbacks

void tud_mount_cb(void)
{
    printf("[USB] Mounted!\n");
    s_blink_interval_ms = BLINK_MOUNTED;
    printf("Files at root:\n");
    DIR d;
    FILINFO fno;
    FRESULT fr = f_findfirst(&d, &fno, "/", "*");
    while (fr == FR_OK && fno.fname[0])
    {
        printf("--%s\n", fno.fname);
        fr = f_findnext(&d, &fno);
    }

}

void tud_umount_cb(void)
{
    printf("[USB] Umounted!\n");
    s_blink_interval_ms = BLINK_NOT_MOUNTED;
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    printf("[USB] Suspended!\n");
    (void)remote_wakeup_en;
    s_blink_interval_ms = BLINK_SUSPENDED;
}

void tud_resume_cb(void)
{
    printf("[USB] Resumed!\n");
    s_blink_interval_ms = BLINK_MOUNTED;
}

#if defined(__CC_ARM)
uint16_t __builtin_bswap16(uint16_t u16)
{
    return __REVSH(u16);
}
#endif // defined(__CC_ARM)
