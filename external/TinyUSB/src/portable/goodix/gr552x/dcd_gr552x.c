/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"
#include "device/dcd.h"
#include "tusb_types.h"

#if CFG_TUSB_MCU == OPT_MCU_GR552X
#include "grx_sys.h"
#include "gr_soc.h"
#include "grx_hal.h"

usb_handle_t g_usb_handle;

bool g_usb_reset_state = false;
static bool s_ep0_tx_non_empty = false;
static void handle_ep0_tx(void);

void USB_IRQHandler(void);
void USB_ATTACH_IRQHandler(void);
void USB_DETACH_IRQHandler(void);
//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
// Transfer Descriptor
#define EP_MAX                    6
#define XFER_CTL_BASE(_ep, _dir)  &_dcd.xfer_status[_ep][_dir]
#define FIFO_SIZE                 64

/* USB EP */
#define USB_EP0                   0
#define USB_EP1                   1
#define USB_EP2                   2
#define USB_EP3                   3
#define USB_EP4                   4
#define USB_EP5                   5

#define USB_EP4_DMA_MODE_EN       1
#define USB_EP5_DMA_MODE_EN       1
typedef struct
{
    uint8_t *buffer;
    // Total length of current transfer
    uint16_t total_len;
    // Bytes transferred so far
    uint16_t transferred;
    uint16_t max_packet_size;
    // Packet size sent or received so far. It is used to modify transferred field
    // after ACK is received or when filling ISO endpoint with size larger then
    // FIFO size.
    uint16_t last_packet_size;
    uint8_t ep_addr;
    // DATA0/1 toggle bit 1 DATA1 is expected or transmitted
    uint8_t data1 : 1;
    // Endpoint is stalled
    uint8_t stall : 1;
    // ISO endpoint
    uint8_t iso : 1;
} xfer_ctl_t;

// Data for managing dcd
static struct
{
    bool vbus_present;
    bool init_called;
    uint8_t nfsr;
    xfer_ctl_t xfer_status[EP_MAX][2];
    // Endpoints that use DMA, one for each direction
    uint8_t dma_ep[2];
} _dcd = {
    .vbus_present = false,
    .init_called = false,
};

// check if we are in ISR
TU_ATTR_ALWAYS_INLINE static bool is_in_isr(void)
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) ? true : false;
}

static inline void usb_tx_zlp(void)
{
    hal_usb_set_cmd_ok(&g_usb_handle);
    return;
}

static inline void usbd_ack(void)
{
    hal_usb_set_cmd_ok(&g_usb_handle);
    return;
}

static inline void usbd_set_cmd_err(void)
{
    hal_usb_set_cmd_err(&g_usb_handle);
    return;
}

void usb_enable_attach_detach(void)
{
    /* Enable the USB peripheral */
    __HAL_USB_ENABLE();
    /*enable usb attach interrupt*/
    NVIC_ClearPendingIRQ(USB_ATTACH_IRQn);
    NVIC_EnableIRQ(USB_ATTACH_IRQn);
    /*enable usb detech interrupt*/
    NVIC_ClearPendingIRQ(USB_DETACH_IRQn);
    NVIC_EnableIRQ(USB_DETACH_IRQn);
}
/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init(uint8_t rhport)
{
    soc_register_nvic(USB_IRQn, (uint32_t)USB_IRQHandler);
    soc_register_nvic(USB_ATTACH_IRQn, (uint32_t)USB_ATTACH_IRQHandler);
    soc_register_nvic(USB_DETACH_IRQn, (uint32_t)USB_DETACH_IRQHandler);

    _dcd.init_called = true;
    _dcd.xfer_status[0][0].max_packet_size = 64;
    _dcd.xfer_status[0][1].max_packet_size = 64;

    g_usb_handle.p_instance = USB;
    g_usb_handle.init.speed = USB_HAL_SPEED_FULL;
    // software enumeration
    g_usb_handle.init.enum_type = USB_ENUM_TYPE_MCU;
    hal_usb_deinit(&g_usb_handle);
    if(PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get())
    {
        hal_usb_init(&g_usb_handle);
    }
    else
    {   /* In sleep mode, call hal_usb_init when USB attach */
        usb_enable_attach_detach();
    }

    NVIC_SetPriority(USB_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 15));
}

void dcd_deinit(void)
{
    WRITE_REG(USB->INT_EN, 0);
    WRITE_REG(USB->INT_CLR, USB_INT_CLR_ALL);
    WRITE_REG(USB->INT_EN, 0);
    hal_usb_deinit(&g_usb_handle);
    NVIC_DisableIRQ(USB_IRQn);
    NVIC_DisableIRQ(USB_ATTACH_IRQn);
    NVIC_DisableIRQ(USB_DETACH_IRQn);
    NVIC_ClearPendingIRQ(USB_IRQn);
    NVIC_ClearPendingIRQ(USB_ATTACH_IRQn);
    NVIC_ClearPendingIRQ(USB_DETACH_IRQn);
}

// Enable device interrupt
void dcd_int_enable(uint8_t rhport)
{
    (void)rhport;
    NVIC_EnableIRQ(USB_IRQn);
}

// Disable device interrupt
void dcd_int_disable(uint8_t rhport)
{
    (void)rhport;
    NVIC_DisableIRQ(USB_IRQn);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    (void)rhport;
    hal_usb_set_addr(&g_usb_handle, dev_addr);
    dcd_edpt_xfer(rhport, 0 | TUSB_DIR_IN_MASK, NULL, 0);
}

// Wake up host
void dcd_remote_wakeup(uint8_t rhport)
{
    (void)rhport;
    __HAL_USB_ENABLE_DEV_REMOTE_WAKEUP(&g_usb_handle);
    ll_usb_enable_mcu_wakeup(USB);
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
    (void)rhport;
    ll_usb_disable_xcvr_dm_rpu(USB);
    ll_usb_enable_xcvr_dp_rpu(USB);
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
    (void)rhport;
    ll_usb_disable_xcvr_dm_rpu(USB);
    ll_usb_disable_xcvr_dp_rpu(USB);
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_edpt)
{
    (void)rhport;

    uint8_t epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
    uint8_t const dir = tu_edpt_dir(desc_edpt->bEndpointAddress);
    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, dir);

    TU_ASSERT(epnum < EP_MAX);

    xfer->max_packet_size = tu_edpt_packet_size(desc_edpt);
    xfer->ep_addr = desc_edpt->bEndpointAddress;
    xfer->data1 = 0;
    xfer->iso = 0;

    uint32_t attr = USB_EP_ATTR_INT;
    switch (desc_edpt->bmAttributes.xfer)
    {
    case TUSB_XFER_ISOCHRONOUS:
        attr = USB_EP_ATTR_ISO;
        break;
    case TUSB_XFER_INTERRUPT:
        attr = USB_EP_ATTR_INT;
        break;
    case TUSB_XFER_BULK:
        attr = USB_EP_ATTR_BULK;
        break;
    default:
        attr = USB_EP_ATTR_INT;
        break;
    }

    switch (epnum)
    {
    case USB_EP4:
#if USB_EP4_DMA_MODE_EN
        ll_usb_enable_it(g_usb_handle.p_instance, LL_USB_INT_EN_EP4_AHB_XFER_DONE);
        ll_usb_disable_ep4_ahb_m(g_usb_handle.p_instance);
#else
        ll_usb_enable_it(g_usb_handle.p_instance, LL_USB_INT_EN_EP4_TX_DONE);
        ll_usb_disable_ep4_dat_rdy(g_usb_handle.p_instance);
#endif
        ll_usb_enable_clr_ep4_fifo(USB);
        break;
    case USB_EP5:
#if USB_EP5_DMA_MODE_EN
        __HAL_USB_SET_EP5_TIMER_VAL(&g_usb_handle, 0);
        __HAL_USB_DISABLE_IT(&g_usb_handle, USB_IT_EP5_OUT_READY);
        __HAL_USB_ENABLE_IT(&g_usb_handle, USB_IT_EP5_AHB_XFER_DONE | USB_IT_EP5_TIMER_OUT_ERR);
        ll_usb_enable_ep5_fifo_clr(USB);
        __HAL_USB_ENABLE_EP5_DMA_READ(&g_usb_handle);
#else
        __HAL_USB_ENABLE_IT(&g_usb_handle, USB_IT_EP5_OUT_READY);
#endif
    }
    TU_LOG2("%s epnum %d\r\n",__FUNCTION__, epnum);
    TU_LOG2("epnum %d, attr %d\r\n", epnum, attr);
    hal_usb_set_and_unhalt_ep(&g_usb_handle, (hal_usb_ep_t)epnum, attr);
    return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir = tu_edpt_dir(ep_addr);
    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, dir);
    TU_LOG2("%s epnum %d\r\n",__FUNCTION__, epnum);
    switch (epnum)
    {
    case USB_EP1:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP1);
        break;
    case USB_EP2:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP2);
        break;
    case USB_EP3:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP3);
        break;
    case USB_EP4:
        if ((ll_usb_is_enabled_ep4_ahb_m(g_usb_handle.p_instance)))
        {
            TU_LOG2("%s ERROR: EP4 DMA is busy, halt EP4 FORCE \r\n", __FUNCTION__);
        }
        break;
    case USB_EP5:
#if USB_EP5_DMA_MODE_EN
        __HAL_USB_SET_EP5_TIMER_VAL(&g_usb_handle, 1);
        __HAL_USB_DISABLE_IT(&g_usb_handle, USB_IT_EP5_AHB_XFER_DONE | USB_IT_EP5_TIMER_OUT_ERR | USB_IT_EP5_OUT_READY);
#else
        __HAL_USB_DISABLE_IT(&g_usb_handle, USB_IT_EP5_OUT_READY);
#endif
        ll_usb_enable_ep5_fifo_clr(USB);
        __HAL_USB_DISABLE_EP5_DMA_READ(&g_usb_handle);
        break;
    default:

        break;
    }
    hal_usb_halt_ep(&g_usb_handle, (hal_usb_ep_t)epnum);
    tu_memclr(xfer, sizeof(*xfer));
}

void dcd_edpt_close_all(uint8_t rhport)
{
    TU_LOG2("%s\r\n", __FUNCTION__);
    hal_usb_halt_ep(&g_usb_handle, HAL_USB_EP1);
    hal_usb_halt_ep(&g_usb_handle, HAL_USB_EP2);
    hal_usb_halt_ep(&g_usb_handle, HAL_USB_EP3);
    hal_usb_halt_ep(&g_usb_handle, HAL_USB_EP4);
    hal_usb_halt_ep(&g_usb_handle, HAL_USB_EP5);
    (void)rhport;
}

static void usb_ep_dma_write(int ep, xfer_ctl_t *xfer)
{
    if(ep == USB_EP3 || ep == USB_EP4)
    {
        uint16_t left_to_send;
        uint8_t *src;
        uint8_t const epnum = tu_edpt_number(xfer->ep_addr);

        src = &xfer->buffer[xfer->transferred];
        left_to_send = xfer->total_len - xfer->transferred;
        if (left_to_send > xfer->max_packet_size)
        {
            left_to_send = xfer->max_packet_size;
        }
        xfer->last_packet_size = left_to_send;

        if(left_to_send == 0)
        {
            return;
        }

        hal_status_t res = HAL_OK;
        if(ep == USB_EP3)
        {
            if(ll_usb_is_enabled_ep3_ahb_m(USB))
            {
                TU_LOG1("WARRING: EP3 DMA IS BUSY. Wait\r\n");
                while(ll_usb_is_enabled_ep3_ahb_m(USB));
            }
        }
        else if(ep == USB_EP4)
        {
            if(ll_usb_is_enabled_ep4_ahb_m(USB))
            {
                TU_LOG1("WARRING: EP4 DMA IS BUSY. Wait\r\n");
                while(ll_usb_is_enabled_ep4_ahb_m(USB));
            }
        }

        res = hal_usb_ep_transmit_dma(&g_usb_handle,(hal_usb_ep_t)ep, src, left_to_send);
        if(res == HAL_OK)
        {

        }
        else if(res == HAL_BUSY)
        {
            TU_LOG1("WARRING: usb_ep[%d] BUSY \r\n",ep);
        }
        else
        {
            TU_LOG1("WARRING: usb_ep[%d] HAL_ERROR \r\n ",ep);
        }
    }
}

static void fill_tx_fifo(xfer_ctl_t * xfer)
{
    uint8_t left_to_send;
    uint8_t const epnum = tu_edpt_number(xfer->ep_addr);

    left_to_send = xfer->total_len - xfer->transferred;
    if (left_to_send > xfer->max_packet_size)
    {
        left_to_send = xfer->max_packet_size;
    }
    xfer->last_packet_size = left_to_send;

    uint8_t *buf = (uint8_t *)&xfer->buffer[xfer->transferred];
    hal_usb_ep_write_start(&g_usb_handle,(hal_usb_ep_t)epnum);

    if (left_to_send)
    {
        int32_t loops = left_to_send;
        uint32_t value;
        uint8_t *pdata = (uint8_t*)buf;
        while (loops > 0)
        {
            if (epnum == USB_EP3)
            {
                hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP3,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_DEFAULT);
                pdata += 4;
                loops -= 4;
            }
            else if(epnum == USB_EP4)
            {
                if(loops >= 4)
                {
                    hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_4BYTE);
                    pdata += 4;
                    loops -= 4;
                }
                else
                {   switch(loops)
                    {
                        case 1:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_1BYTE);
                            break;
                        case 2:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_2BYTE);
                            break;
                        case 3:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_3BYTE);
                            break;
                        default:
                            break;
                    }
                    loops = 0;
                }
            }
            else //EP0 EP2
            {
                value = (uint32_t)(*pdata);
                hal_usb_write_ep_fifo(&g_usb_handle, (hal_usb_ep_t)epnum, value, USB_EP4_FIFO_WEN_DEFAULT);
                pdata ++;
                loops--;
            }
        }
    }
    /*MCU has write all data to IN FIFO, then inform USB controller */
    hal_usb_ep_write_end(&g_usb_handle,(hal_usb_ep_t)epnum);
}

static void usbd_ep_write(xfer_ctl_t * xfer)
{
    uint8_t const epnum = tu_edpt_number(xfer->ep_addr);
    if(((epnum == USB_EP3) && (xfer->total_len >= xfer->max_packet_size))
        ||((epnum == USB_EP3) && (xfer->total_len < xfer->max_packet_size) && (xfer->total_len % 4 != 0)))  //EP3 FIFO write 4byte every time
    {
        usb_ep_dma_write(epnum, xfer);
    }
    else if(epnum == USB_EP4)
    {
        usb_ep_dma_write(epnum, xfer);
        if(xfer->total_len == 0)
        {
            dcd_event_xfer_complete(0, USB_EP4 | TUSB_DIR_IN_MASK, 0, XFER_RESULT_SUCCESS, is_in_isr());
        }
    }
    else
    {
        fill_tx_fifo(xfer);
    }
}

void read_ep0_data(void)
{
    ll_usb_disable_ep0_out_dat_rdy(USB);
    usbd_ack();
    while(!(__HAL_USB_GET_EP0_OUT_DAT_RDY(&g_usb_handle)));

    uint8_t ep0_buffer[64];
    uint32_t fifo_bytes;
    hal_status_t   ret =HAL_OK;

    ret = hal_usb_ep_read_start(&g_usb_handle, HAL_USB_EP0);
    if (HAL_OK != ret)
    {
       TU_LOG1("ERROR: hal_usb_ep_read_start ret %d \r\n", ret);
    }
    fifo_bytes = hal_usb_get_ep0_rx_data_sum(&g_usb_handle); // byte

    memset(ep0_buffer,0,sizeof(ep0_buffer));
    uint32_t loops = ((fifo_bytes + 3) >> 2);

    uint32_t *ptr = (uint32_t *)ep0_buffer;
    for (int i = 0; i < loops; i++)
    {
        ptr[i] = hal_usb_read_ep_fifo(&g_usb_handle, HAL_USB_EP0); // word
    }

    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP0, TUSB_DIR_OUT);
    uint8_t *buf = xfer->buffer;
    memcpy(buf, ep0_buffer, fifo_bytes);
    xfer->last_packet_size = fifo_bytes;
    xfer->transferred += xfer->last_packet_size;
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    if (xfer->transferred == xfer->total_len)
    {
        dcd_event_xfer_complete(0, USB_EP0, xfer->total_len, XFER_RESULT_SUCCESS, is_in_isr());
    }
    ll_usb_disable_ep0_out_dat_rdy(USB);
}

//HOST will OUT data
static void start_rx_packet(xfer_ctl_t *xfer)
{
    uint8_t const epnum = tu_edpt_number(xfer->ep_addr);
    uint16_t remaining = xfer->total_len - xfer->transferred;
    uint16_t size = tu_min16(remaining, xfer->max_packet_size);

    xfer->last_packet_size = 0;
    if(epnum == 0)
    {
       if(xfer->total_len == 0)
       {
           //STATUS STAGE : Device ACK HOST
            dcd_event_xfer_complete(0, USB_EP0, 0, XFER_RESULT_SUCCESS, is_in_isr());
            usbd_ack();
       }
       else{
            read_ep0_data();
       }
    }
    if(epnum == 1)
    {
        ll_usb_disable_ep1_out_dat_rdy(USB);
    }
    if(epnum == 5)
    {
    #if USB_EP5_DMA_MODE_EN
        __HAL_USB_DISABLE_EP5_DMA_READ(&g_usb_handle);
        hal_usb_ep_receive_dma(&g_usb_handle,HAL_USB_EP5, xfer->buffer, xfer->total_len);
        ll_usb_enable_ep5_fifo_clr(USB);
        __HAL_USB_ENABLE_EP5_DMA_READ(&g_usb_handle);
    #endif
    }
}

//HOST IN data
static void start_tx_packet(xfer_ctl_t *xfer)
{
    uint8_t const epnum = tu_edpt_number(xfer->ep_addr);
    if((epnum == 0) && (xfer->total_len == 0))
    {
        //status satge
        dcd_event_xfer_complete(0, USB_EP0 | TUSB_DIR_IN_MASK, 0, XFER_RESULT_SUCCESS, is_in_isr());
        usb_tx_zlp();
        return;
    }
    else
    {
        if(epnum == 0)
        {
            s_ep0_tx_non_empty =1;
        }
        usbd_ep_write(xfer);
    }
}

static TU_ATTR_ALIGNED(4) uint8_t _setup_packet[8];
static void handle_ep0_rx(void)
{
    uint32_t fifo_bytes;
    hal_status_t   ret =HAL_OK;

    ret = hal_usb_ep_read_start(&g_usb_handle, HAL_USB_EP0);
    if (HAL_OK != ret)
    {
       TU_LOG1("ERROR: hal_usb_ep_read_start ret %d \r\n", ret);
    }
    fifo_bytes = hal_usb_get_ep0_rx_data_sum(&g_usb_handle); // byte
    if(fifo_bytes != 8)
    {
       TU_LOG1("ERROR: hal_usb_ep_read_start ret %d \r\n", ret);
       return;
    }
    memset(_setup_packet,0,sizeof(_setup_packet));

    uint32_t loops = ((fifo_bytes + 3) >> 2);

    uint32_t *ptr = (uint32_t *)_setup_packet;
    for (int i = 0; i < loops; i++)
    {
        ptr[i] = hal_usb_read_ep_fifo(&g_usb_handle, HAL_USB_EP0); // word
    }
    ret = hal_usb_ep_read_end(&g_usb_handle, HAL_USB_EP0);
    if (HAL_OK != ret)
    {
        TU_LOG1("ERROR: SS hal_usb_ep_read_end ret %d \r\n", ret);
    }

    xfer_ctl_t *xfer_in = XFER_CTL_BASE(USB_EP0, TUSB_DIR_IN);
    // Setup packet is in
    xfer_in->stall = 0;
    xfer_in->data1 = 1;
    xfer_in->data1 ^= 1;
    dcd_event_setup_received(0, _setup_packet, true);
}

static void handle_ep0_tx(void) //TX DONE IRQ
{
    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP0, TUSB_DIR_IN);

    xfer->transferred += xfer->last_packet_size;
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    if (xfer->transferred == xfer->total_len)
    {
        dcd_event_xfer_complete(0, USB_EP0 | TUSB_DIR_IN_MASK, xfer->transferred, XFER_RESULT_SUCCESS, is_in_isr());
    }
    else
    {
        //tx letf data
        TU_LOG1("handle_ep0_tx: not complete\r\n");
    }
}

static void handle_ep1_rx(void)
{
    uint8_t fifo_bytes;

    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP1, TUSB_DIR_OUT);
    /*MCU has receive all data from OUT FIFO, then clear out packet ready*/
    hal_usb_ep_read_start(&g_usb_handle, HAL_USB_EP1);
    fifo_bytes = hal_usb_get_ep1_rx_data_sum(&g_usb_handle); // byte

    uint8_t ep1_buffer[64];

    uint32_t loops = ((fifo_bytes + 3) >> 2);

    uint32_t *ptr = (uint32_t *)ep1_buffer;
    for (int i = 0; i < loops; i++)
    {
        ptr[i] = hal_usb_read_ep_fifo(&g_usb_handle, HAL_USB_EP1); // word
    }

    uint8_t *buf = xfer->buffer + xfer->transferred;
    if((xfer->transferred + xfer->last_packet_size) < 64 )
    {
        memcpy(buf, ep1_buffer, fifo_bytes);
    }
    else
    {
        TU_LOG1("EP1 FIFO FULL \r\n");
    }
    xfer->last_packet_size = fifo_bytes;
    xfer->data1 ^= 1;
    xfer->transferred += xfer->last_packet_size;
    dcd_event_xfer_complete(0, USB_EP1, xfer->transferred, XFER_RESULT_SUCCESS, is_in_isr());
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
    (void) rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);
    xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
    xfer->buffer = buffer;
    xfer->total_len = total_bytes;
    xfer->last_packet_size = 0;
    xfer->transferred = 0;
    xfer->data1 = 0;
    if (dir == TUSB_DIR_OUT)
    {
        start_rx_packet(xfer);
    }
    else
    {
        start_tx_packet(xfer);
    }

    return true;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
    TU_LOG1("dcd_edpt_xfer_fifo \r\n");
    (void) rhport;
    (void) ep_addr;
    (void) ff;
    (void) total_bytes;
    return false;
}

// Stall endpoint
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    switch(epnum)
    {
    case USB_EP0:
    case USB_EP0 | TUSB_DIR_IN_MASK:
        usbd_set_cmd_err();
        break;
    case USB_EP1:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP1);
        break;
    case USB_EP2:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP2);
        break;
    case USB_EP3:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP3);
        break;
    case USB_EP4:
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP4);
        break;
    case USB_EP5:
#if USB_EP5_DMA_MODE_EN
        __HAL_USB_SET_EP5_TIMER_VAL(&g_usb_handle,1);
        __HAL_USB_DISABLE_IT(&g_usb_handle,USB_IT_EP5_AHB_XFER_DONE  | USB_IT_EP5_TIMER_OUT_ERR | USB_IT_EP5_OUT_READY);
#else
        __HAL_USB_DISABLE_IT(&g_usb_handle,USB_IT_EP5_OUT_READY);
#endif
         ll_usb_enable_ep5_fifo_clr(USB);
        __HAL_USB_DISABLE_EP5_DMA_READ(&g_usb_handle);
        hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP5);
        break;
    default:
        break;
    }
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    usbd_ack();
}

void USB_IRQHandler(void)
{
    hal_usb_irq_handler(&g_usb_handle);
}

void USB_ATTACH_IRQHandler(void)
{
    hal_usb_init(&g_usb_handle);
    hal_usb_attach_irq_handler(&g_usb_handle);
}

void USB_DETACH_IRQHandler(void)
{
    hal_usb_deinit(&g_usb_handle);

    hal_usb_detach_irq_handler(&g_usb_handle);
    dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, true);
    g_usb_reset_state = false;
    usb_enable_attach_detach();
}

void hal_usb_sof_callback(usb_handle_t *p_usb)
{
    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
}

void hal_usb_host_reset_callback(usb_handle_t *p_usb)
{
    if(!g_usb_reset_state)
    {
        dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
        g_usb_reset_state = true;
    }
}

void hal_usb_suspend_callback(usb_handle_t *p_usb)
{
    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
}

//HOST tx , data in EP0 FIFO
void hal_usb_ep0_out_ready_callback(usb_handle_t *p_usb)
{
    handle_ep0_rx();
}

//HOST received data     // ACK received
void hal_usb_ep0_tx_done_callback(usb_handle_t *p_usb)
{
    if(s_ep0_tx_non_empty)
    {
        handle_ep0_tx();
        s_ep0_tx_non_empty=0;
    }
}


void hal_usb_ep1_out_ready_callback(usb_handle_t *p_usb)
{
    handle_ep1_rx();
}

void hal_usb_ep2_tx_done_callback(usb_handle_t *p_usb)
{
    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP2, TUSB_DIR_IN);
    xfer->transferred += xfer->last_packet_size;
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    dcd_event_xfer_complete(0, USB_EP2 | TUSB_DIR_IN_MASK, xfer->total_len, XFER_RESULT_SUCCESS, is_in_isr());
}

void hal_usb_ep3_tx_done_callback(usb_handle_t *p_usb)
{
    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP3, TUSB_DIR_IN);
    xfer->transferred += xfer->last_packet_size;
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    dcd_event_xfer_complete(0, USB_EP3 | TUSB_DIR_IN_MASK, xfer->transferred, XFER_RESULT_SUCCESS, is_in_isr());
}

#if USB_EP4_DMA_MODE_EN
void hal_usb_ep4_ahb_xfer_done_callback(usb_handle_t *p_usb)
{
    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP4, TUSB_DIR_IN);
    xfer->transferred += xfer->last_packet_size;
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    dcd_event_xfer_complete(0, USB_EP4 | TUSB_DIR_IN_MASK, xfer->transferred, XFER_RESULT_SUCCESS, is_in_isr());
}
#else
void hal_usb_ep4_tx_done_callback(usb_handle_t *p_usb)
{
    TU_LOG2("%s\r\n",__FUNCTION__);
}
#endif

#if USB_EP5_DMA_MODE_EN
void hal_usb_ep5_ahb_xfer_done_callback(usb_handle_t *p_usb)
{
    xfer_ctl_t *xfer = XFER_CTL_BASE(USB_EP5, TUSB_DIR_OUT);
    xfer->transferred += hal_usb_get_ep5_rx_data_sum(p_usb);
    xfer->last_packet_size = 0;
    xfer->data1 ^= 1;
    dcd_event_xfer_complete(0, USB_EP5, xfer->transferred, XFER_RESULT_SUCCESS, is_in_isr());

    __HAL_USB_SET_EP5_TIMER_VAL(p_usb,3);
    /*EP5 DMA enable should after EP5 buffer data having been copyed*/
    __HAL_USB_ENABLE_EP5_DMA_READ(p_usb);
}
#else
void hal_usb_ep5_out_ready_callback(usb_handle_t *p_usb)
{
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP5].ep);
}
#endif
void hal_usb_ep5_timer_out_err_callback(usb_handle_t *p_usb)
{
    dcd_event_xfer_complete(0, USB_EP5, 0, XFER_RESULT_FAILED, is_in_isr());
}

void hal_usb_crc16_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_CRC16;
}

 void hal_usb_upid_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_UPID;
}

void hal_usb_time_out_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_TIMEOUT;
}

void hal_usb_seq_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_SEQ;
}

void hal_usb_pid_cks_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_PID_CKS;
}

void hal_usb_pid_crc_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_PID_CRC;
}

void hal_usb_ahb_xfer_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_DMA_RX;
}

void hal_usb_nse_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_NSE;
}

void hal_usb_sync_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_SYNC;
}

void hal_usb_bit_stuff_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_BIT_STUFF;
}

void hal_usb_byte_err_callback(usb_handle_t *p_usb)
{
    TU_LOG1("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_BYTE;
}

#endif /* OPT_MCU_GR552X */
