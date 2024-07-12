# tiny_usbd_msc_fatfs_freertos示例工程说明

## 工程介绍

本工程为GR5526在TinyUSB下使用Mass Storage Class的Example。

在本工程中，会初始化连接到QSPI0的外部Flash，并尝试使用FatFs将整个外部Flash挂载为一个FAT分区，如果挂载失败，则会格式化整个Flash并重新挂载。FatFs的适配在`Src/fatfs/diskio_norflash.c`中。

当FatFs一切正常后，会运行TinyUSB Device的主循环。当USB插入时，TinyUSB会自动根据配置好的Descriptor完成枚举操作。在本工程中，MSC使用了Endpoint 1作为EP Out，Endpoing 3作为EP In。由于MSC要求使用BULK传输，故只能使用EP1/EP2或EP1/EP3组合，下面是GR5526 USB的6个Endpoint的关键参数：

Endpoint | DIrection | Supported Transfer Type | FIFO Size | Functionality | DMA Support
-|-|-|-|-|-
0 | IN/OUT | Control | 2*64Bytes|Enumeration/Config|No
1 | OUT | Bulk/Interrupt | 64Bytes | HOST OUT|No
2 | IN | Bulk/Interrupt | 64Bytes | HOST IN|No
3 | IN | Bulk/Interrupt | 64Bytes | HOST IN|Yes
4 | IN | Isochronous | 1023Bytes | HOST IN|Yes
5 | OUT | Isochronous | 1023Bytes | HOST OUT|Yes

USB与MSC相关的配置在`Src/usbd_user/usb_descriptor.c`中，MSC读写与参数获取的适配在`Src/usbd_user/msc_disk.c`中。

## TinyUSB修改点

本工程对TinyUSB源码做了如下修改：

`msc_device.c`从`604`行开始：

```c
  if ( p_msc->stage == MSC_STAGE_STATUS )
  {
    // skip status if epin is currently stalled, will do it when received Clear Stall request
    if ( !usbd_edpt_stalled(rhport,  p_msc->ep_in) )
    {
#if !TU_CHECK_MCU(OPT_MCU_GR552X) // Stall workaround
      if ( (p_cbw->total_bytes > p_msc->xferred_len) && is_data_in(p_cbw->dir) )
      {
        // 6.7 The 13 Cases: case 5 (Hi > Di): STALL before status
        // TU_LOG(MSC_DEBUG, "  SCSI case 5 (Hi > Di): %lu > %lu\r\n", p_cbw->total_bytes, p_msc->xferred_len);
        usbd_edpt_stall(rhport, p_msc->ep_in);
      }else
#endif // TU_CHECK_MCU(OPT_MCU_GR552X)
      {
        TU_ASSERT( send_csw(rhport, p_msc) );
      }
    }
```

通过`#if !TU_CHECK_MCU(OPT_MCU_GR552x)`宏判断绕开了当`dCBWDataTransferLength`大于实际回复长度时对Endpoint的Stall行为。修改理由是Windows电脑在发送`READ_FORMAT_CAPACITY (0x23)`指令时会传入`dCBWDataTransferLength=252`，而实际回复只有12字节。此时如果按照规范将当前Endpoint Stall，实测后续Windows并不会Reset当前Endpoint，导致通信异常，故绕开该Stall条件。

