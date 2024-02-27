/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#include <stddef.h>
#include <string.h>
#include "gr55xx_usb_server.h"
#include "gr55xx_usb_composite.h"
#include <ctype.h>
#include "gr55xx_usb_audio.h"
#include "grx_hal.h"
#include "gr_soc.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern usb_handle_t g_usb_handle;
extern void USB_IRQHandler(void);
extern void USB_ATTACH_IRQHandler(void);
extern void USB_DETACH_IRQHandler(void);

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
extern void gr55xx_audio_tx_rx_loopback_test(void);
void dma_test_main0(void);
void dma_test_main1(void);

int audio_test(void)
{
    g_usb_handle.p_instance = USB;
    g_usb_handle.init.speed = USB_HAL_SPEED_FULL;

    //software enumeration
    g_usb_handle.init.enum_type = USB_ENUM_TYPE_MCU;

    soc_register_nvic(USB_IRQn, (uint32_t)USB_IRQHandler);
    soc_register_nvic(USB_ATTACH_IRQn, (uint32_t)USB_ATTACH_IRQHandler);
    soc_register_nvic(USB_DETACH_IRQn, (uint32_t)USB_DETACH_IRQHandler);
    hal_usb_init(&g_usb_handle);

    gr55xx_usb_audio_init(&g_usb_handle);

    // dma_test_main0();
    // dma_test_main1();
    while(1)
    {
        gr55xx_audio_tx_rx_loopback_test();
    }
}

