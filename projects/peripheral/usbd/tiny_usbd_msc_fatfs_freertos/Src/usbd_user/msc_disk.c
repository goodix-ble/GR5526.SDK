#include "tusb.h"
#include "drv_adapter_norflash.h"
#include "app_qspi.h"
#include "gr552xx.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SECTOR_SIZE (512) // minimum sector size of FatFS is 512 byte

#if CFG_TUD_MSC

static uint32_t s_disk_sectors = 0;

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4, charatcters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;

    const char vid[] = "GR5526";
    const char pid[] = "MSC Example";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;

    return true;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;

    if (!s_disk_sectors)
    {
        drv_adapter_norflash_set_mmap_mode(false);
        s_disk_sectors = qspi_norf_read_dev_density() / SECTOR_SIZE;
    }

    *block_count = s_disk_sectors;
    *block_size = SECTOR_SIZE;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;

    if (load_eject)
    {
        if (start)
        {
            // load disk storage
        }
        else
        {
            // unload disk storage
        }
    }

    return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;

    if (lba >= s_disk_sectors)
    {
        return -1;
    }

    drv_adapter_norflash_set_mmap_mode(true);
    app_qspi_mmap_set_endian_mode(APP_QSPI_ID_2, APP_QSPI_MMAP_ENDIAN_MODE_2);
    memcpy(buffer, (void *)(QSPI0_XIP_BASE + (lba * SECTOR_SIZE) + offset), bufsize);

    return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;

    if (lba >= s_disk_sectors)
    {
        return -1;
    }

    // This will only work when bufsize == 64!
    drv_adapter_norflash_set_mmap_mode(false);
    uint8_t page_buffer[256];
    uint32_t addr = (lba * SECTOR_SIZE) + offset;
    uint32_t page_addr = addr & 0xFFFFFF00;
    uint32_t page_offset = addr & 0x000000FF;
    // Read
    drv_adapter_norflash_read(page_addr, page_buffer, sizeof(page_buffer));
    // Erase
    bool ret = drv_adapter_norflash_erase(page_addr, ADAPTER_NORFFLASH_ERASE_PAGE);
    // Modify
    memcpy(page_buffer + page_offset, buffer, bufsize);
    // Write
    ret |= drv_adapter_norflash_write(page_addr, page_buffer, sizeof(page_buffer));

    return ret ? bufsize : 0;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    // read10 & write10 has their own callback and MUST not be handled here
    void const *response = NULL;
    int32_t resplen = 0;

    // most scsi handled is input
    bool in_xfer = true;

    switch (scsi_cmd[0])
    {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        // Host is about to read/write etc ... better not to disconnect disk
        resplen = 0;
        break;

    default:
        // Set Sense = Invalid Command Operation
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

        // negative means error -> tinyusb could stall and/or response with failed status
        resplen = -1;
        break;
    }

    // return resplen must not larger than bufsize
    if (resplen > bufsize)
        resplen = bufsize;

    if (response && (resplen > 0))
    {
        if (in_xfer)
        {
            memcpy(buffer, response, resplen);
        }
        else
        {
            // SCSI output
        }
    }

    return resplen;
}

#endif // CFG_TUD_MSC
