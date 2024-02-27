#include "app_qspi.h"
#include "app_log.h"
#include "qspi_psram.h"



/*
 * QSPI DEFINES
 *****************************************************************************************
 */
/*****************************************
 * CHANGE FOLLOWING SETTINGS By YOUR CASE !
 *****************************************/

#define QSPI_CLOCK_PRESCALER             2u                     /* The QSPI CLOCK Freq = Peripheral CLK/QSPI_CLOCK_PRESCALER */
#define QSPI_WAIT_TIMEOUT_MS             1500u                  /* default time(ms) for wait operation */
#define QSPI_ID                          APP_QSPI_ID_1
#define QSPI_PIN_GROUP                   QSPI1_PIN_GROUP_0      /* which pin group to connect */
#define QSPI_TIMING_MODE                 QSPI_CLOCK_MODE_3

/*****************************************
 * CHANGE FOLLOWING SETTINGS CAREFULLY !
 *****************************************/
#if QSPI_ID == APP_QSPI_ID_0
    #define QSPI_USED_DMA                     DMA0
#elif QSPI_ID == APP_QSPI_ID_1
    #define QSPI_USED_DMA                     DMA0
#else
    #define QSPI_USED_DMA                     DMA1
#endif

#if QSPI_CLOCK_PRESCALER == 2u
    #define QSPI_RX_SAMPLE_DELAY         1u
#else
    #define QSPI_RX_SAMPLE_DELAY         0u
#endif

#define DEFAULT_PSRAM_MODE_CONFIG                 {APP_QSPI_TYPE_DMA, QSPI_USED_DMA, DMA_Channel1, 1000, 0}
#define DEFAULT_PSRAM_QSPI_CONFIG                 {QSPI_CLOCK_PRESCALER, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_PSRAM_PARAM_CONFIG                {QSPI_ID, g_qspi_pin_groups[QSPI_PIN_GROUP], DEFAULT_PSRAM_MODE_CONFIG, DEFAULT_PSRAM_QSPI_CONFIG}

static  app_qspi_params_t g_qspi_psram_params;


uint32_t qspi_psram_init(app_qspi_id_t id, app_qspi_access_type_e work_mode, uint32_t clock_prescaler, qspi_pins_group_e pin_group)
{
    uint32_t psram_id = 0;
    uint32_t retry = 20;
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PSRAM_PARAM_CONFIG;

    g_qspi_psram_params = p_params;

    g_qspi_psram_params.id                    = id;
    g_qspi_psram_params.work_mode.access_type = work_mode;
    g_qspi_psram_params.pin_cfg               = g_qspi_pin_groups[pin_group];
    g_qspi_psram_params.init.clock_prescaler  = clock_prescaler;
    if(clock_prescaler == 2){
        g_qspi_psram_params.init.rx_sample_delay = 1;
    }  else {
        g_qspi_psram_params.init.rx_sample_delay = 0;
    }

    if(id == APP_QSPI_ID_2) {
        g_qspi_psram_params.work_mode.dma_instance = DMA1;
    }

    ret = app_qspi_init(&g_qspi_psram_params, NULL);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial failed! Please check the input paraments.");
        return 1;
    }

    while(retry--)  {
        qspi_psram_exit_quad_mode(g_qspi_psram_params.id);
        delay_us(5);
        qspi_psram_reset(g_qspi_psram_params.id);

        delay_us(5);

        psram_id = qspi_psram_read_id(g_qspi_psram_params.id);

        if( (psram_id != 0x0000) && (psram_id != 0xffff) ) {
            qspi_psram_enable_quad_mode(g_qspi_psram_params.id);
            break;
        }
        delay_us(10);
    }

    return psram_id;
}

void qspi_psram_deinit(app_qspi_id_t id)
{
    app_qspi_deinit(id);
}


void qspi_psram_reset(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_RESET_ENABLE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    app_qspi_command(id, &command, APP_QSPI_TYPE_DEFAULT);

    command.instruction = PSRAM_CMD_RESET;
    app_qspi_command(id, &command, APP_QSPI_TYPE_DEFAULT);

    return;
}

void qspi_psram_enable_quad_mode(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_ENTER_QUAD_MODE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    app_qspi_command(id, &command, APP_QSPI_TYPE_DEFAULT);

    return;
}

void qspi_psram_exit_quad_mode(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_EXIT_QUAD_MODE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPIFRF,
        .data_mode                  = QSPI_DATA_MODE_QUADSPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    app_qspi_command(id, &command, APP_QSPI_TYPE_DEFAULT);
    return;
}

uint32_t qspi_psram_read_id(app_qspi_id_t id)
{
    uint8_t data[2];
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_READID,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_24_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 2,
        .clock_stretch_en           = 1,
    };

    memset(&data[0], 0, 2);
    app_qspi_command_receive(id, &command, &data[0], APP_QSPI_TYPE_DEFAULT);

    return (((uint32_t)data[0] << 8) + ((uint32_t)data[1] << 0) );
}

#define MAX_UI_FB_SIZE          (300*1024u)



bool qspi_psram_init_as_xip(void) {
    uint8_t psram_id = 0;

    psram_id = qspi_psram_init(QSPI_ID, APP_QSPI_TYPE_DMA, QSPI_CLOCK_PRESCALER, QSPI_PIN_GROUP);
    printf("PSRAM ID: 0x%x \r\n", psram_id);

    app_qspi_mmap_device_t dev = {
        .dev_type = APP_QSPI_DEVICE_PSRAM,
        .rd.psram_rd = PSRAM_MMAP_CMD_QREAD_0BH,
        .psram_wr    = PSRAM_MMAP_CMD_QWRITE_38H,
    };

    bool ret = app_qspi_config_memory_mappped(QSPI_ID, dev);
    if(ret) {
        uint8_t * ptr = (uint8_t *) lv_port_disp_buff();
        
        memset((void*)ptr, 0, 390*390*2);
        
        qspi_psram_write(15*MAX_UI_FB_SIZE, ptr, 390*390*2);
    }
    
    return ret;
}

bool qspi_psram_write(uint32_t address, uint8_t * data, uint32_t len) {
    
#if 0
    memcpy((void*)(QSPI1_XIP_BASE + address), data, len);
#else
    uint8_t ret = 0;
    app_qspi_command_t wr_command = {
        .instruction      = PSRAM_CMD_QUAD_WRITE,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = len,
        .clock_stretch_en = 1,
    };

    ret += app_qspi_active_memory_mappped(QSPI_ID, false);
    //printf("ret - %d \r\n",ret);
    ret += app_qspi_psram_write(QSPI_ID, &wr_command, data);
    //printf("ret - %d \r\n",ret);
    ret += app_qspi_active_memory_mappped(QSPI_ID, true);
    //printf("ret - %d \r\n",ret);
    if(ret == 3) {
        //printf("qspi_psram_write addr:%08x, len: %d \r\n",address, len);
    } else {
        printf("qspi_psram_write fail ...\r\n");
    }
#endif
    return true;
}
