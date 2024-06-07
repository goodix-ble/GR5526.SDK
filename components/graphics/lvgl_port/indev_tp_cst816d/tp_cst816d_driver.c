#include "tp_cst816d_driver.h"
#include "app_i2c.h"
#include "app_io.h"

#define TP_RST_IO_TYPE APP_IO_TYPE_GPIOB
#define TP_RST_IO_PIN APP_IO_PIN_13
#define TP_RST_IO_MUX APP_IO_MUX

#define TP_INT_IO_TYPE APP_IO_TYPE_GPIOC
#define TP_INT_IO_PIN APP_IO_PIN_0
#define TP_INT_IO_MUX APP_IO_MUX

#define TP_SCL_IO_TYPE APP_IO_TYPE_GPIOB
#define TP_SCL_IO_PIN APP_IO_PIN_14
#define TP_SCL_IO_MUX APP_IO_MUX_3

#define TP_SDA_IO_TYPE APP_IO_TYPE_GPIOB
#define TP_SDA_IO_PIN APP_IO_PIN_15
#define TP_SDA_IO_MUX APP_IO_MUX_3

#define TP_I2C_ID APP_I2C_ID_1

#define TP_I2C_ADDR (0x15)

#define TP_I2C_OPER_TIMEOUT (1000)

static uint16_t tp_i2c_mem_read(uint8_t reg_addr, uint8_t *buf, uint16_t len);
static uint16_t tp_i2c_mem_write(uint8_t reg_addr, uint8_t *data, uint16_t len);
static void tp_int_event_cb(app_io_evt_t *p_evt);

static app_i2c_params_t s_i2c_params = {
    .id = TP_I2C_ID,
    .role = APP_I2C_ROLE_MASTER,
    .pin_cfg = {
        .scl = {
            .type = TP_SCL_IO_TYPE,
            .mux = TP_SCL_IO_MUX,
            .pin = TP_SCL_IO_PIN,
            .pull = APP_IO_PULLUP,
        },
        .sda = {
            .type = TP_SDA_IO_TYPE,
            .mux = TP_SDA_IO_MUX,
            .pin = TP_SDA_IO_PIN,
            .pull = APP_IO_PULLUP,
        },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA1,
        .rx_dma_instance = DMA1,
        .tx_dma_channel = DMA_Channel0,
        .rx_dma_channel = DMA_Channel1,
    },
    .init = {
        .speed = I2C_SPEED_400K,
        .own_address = 0,
        .addressing_mode = I2C_ADDRESSINGMODE_7BIT,
        .general_call_mode = I2C_GENERALCALL_DISABLE,
        .tx_hold_time = 0,
        .rx_hold_time = 0,
    },
};

/**
 * Global Functions
 */

void tp_cst816d_init(tp_int_evt_cb_t cb)
{
    // Initialize I2C
    app_i2c_init(&s_i2c_params, NULL);

    // Initialize corresponding IO
    // Reset
    app_io_init_t io_init;
    io_init.pin = TP_RST_IO_PIN;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pull = APP_IO_NOPULL;
    io_init.mux = TP_RST_IO_MUX;
    app_io_init(TP_RST_IO_TYPE, &io_init);

    // TODO: TP Int pin
    io_init.pin = TP_INT_IO_PIN;
    io_init.mode = APP_IO_MODE_IT_FALLING;
    io_init.pull = APP_IO_PULLUP;
    io_init.mux = TP_INT_IO_MUX;
    app_io_event_register_cb(TP_INT_IO_TYPE, &io_init, tp_int_event_cb, (void *)cb);

    // Reset
    app_io_write_pin(TP_RST_IO_TYPE, TP_RST_IO_PIN, APP_IO_PIN_RESET);
    delay_ms(20);
    app_io_write_pin(TP_RST_IO_TYPE, TP_RST_IO_PIN, APP_IO_PIN_SET);
    delay_ms(200);
}

void tp_cst816d_deinit(void)
{
    app_i2c_deinit(TP_I2C_ID);
}

bool tp_cst816d_read_pointer(int16_t *px, int16_t *py)
{
    uint8_t data[5];

    uint16_t ret = tp_i2c_mem_read(0x02, data, sizeof(data));
    if (APP_DRV_SUCCESS != ret)
    {
        printf("TP Read Failed: %d\n", ret);
        return false;
    }

    if (data[0] == 0 || data[0] > 2)
    {
        return false;
    }
    else
    {
        *px = ((uint16_t)((data[1] & 0x0F)) << 8) | data[2];
        *py = ((uint16_t)((data[3] & 0x0F)) << 8) | data[4];
        return true;
    }
}

bool tp_cst816d_sleep(void)
{
    uint8_t data[] = {0x03};
    return tp_i2c_mem_write(0xE5, data, sizeof(data)) == APP_DRV_SUCCESS;
}

bool tp_cst816d_wakeup(void)
{
    app_io_write_pin(TP_RST_IO_TYPE, TP_RST_IO_PIN, APP_IO_PIN_RESET);
    delay_ms(10);
    app_io_write_pin(TP_RST_IO_TYPE, TP_RST_IO_PIN, APP_IO_PIN_SET);
    return true;
}

/**
 * Static Functions
 */

static uint16_t tp_i2c_mem_read(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    return app_i2c_mem_read_sync(TP_I2C_ID, TP_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, len, TP_I2C_OPER_TIMEOUT);
}

static uint16_t tp_i2c_mem_write(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return app_i2c_mem_write_sync(TP_I2C_ID, TP_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, TP_I2C_OPER_TIMEOUT);
}

static void tp_int_event_cb(app_io_evt_t *p_evt)
{
    tp_int_evt_cb_t cb = (tp_int_evt_cb_t)p_evt->arg;
    if (cb)
    {
        cb();
    }
}
