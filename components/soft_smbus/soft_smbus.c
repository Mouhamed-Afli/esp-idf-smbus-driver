#include <stdlib.h>
#include <string.h>
#include "soft_smbus.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SOFT_SMBUS";

#define SMBUS_BLOCK_MAX      32      /* SMBus 2.0 max block length */
#define BQ_MAC_CMD           0x44    /* ManufacturerBlockAccess */
#define BQ_MAC_DELAY_MS      2

struct soft_smbus_obj_t {
    gpio_num_t sda_io;
    gpio_num_t scl_io;
    uint32_t half_period_us;
    int64_t timeout_us;
    bool pec;
};

/*
 * Pins are configured as open-drain input/output, so:
 *   level 1 -> line released (pulled up), level 0 -> line driven low.
 * No direction switching is needed, which removes the costly
 * gpio_set_direction() calls from the bit loop.
 */
#define SDA_LOW(b)   gpio_set_level((b)->sda_io, 0)
#define SDA_HIGH(b)  gpio_set_level((b)->sda_io, 1)
#define SDA_GET(b)   gpio_get_level((b)->sda_io)
#define SCL_LOW(b)   gpio_set_level((b)->scl_io, 0)
#define DELAY(b)     esp_rom_delay_us((b)->half_period_us)

static esp_err_t scl_high(soft_smbus_handle_t bus)
{
    gpio_set_level(bus->scl_io, 1);

    /* Fast path: no clock stretching (the common case). */
    if (gpio_get_level(bus->scl_io)) {
        return ESP_OK;
    }

    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(bus->scl_io)) {
        if (esp_timer_get_time() - start > bus->timeout_us) {
            ESP_LOGE(TAG, "SCL clock stretching timeout");
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(1);
    }
    return ESP_OK;
}

/* START or repeated START. Entry: SCL low (repeated) or bus idle. */
static esp_err_t i2c_start(soft_smbus_handle_t bus)
{
    SDA_HIGH(bus);
    DELAY(bus);                       /* setup time before raising SCL */
    if (scl_high(bus) != ESP_OK) return ESP_ERR_TIMEOUT;
    DELAY(bus);
    SDA_LOW(bus);
    DELAY(bus);
    SCL_LOW(bus);
    return ESP_OK;
}

/* STOP. Entry: SCL low. */
static esp_err_t i2c_stop(soft_smbus_handle_t bus)
{
    SDA_LOW(bus);
    DELAY(bus);
    esp_err_t ret = scl_high(bus);
    DELAY(bus);
    SDA_HIGH(bus);
    DELAY(bus);
    return ret;
}

static esp_err_t i2c_write_byte(soft_smbus_handle_t bus, uint8_t data)
{
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        gpio_set_level(bus->sda_io, (data & mask) ? 1 : 0);
        DELAY(bus);
        if (scl_high(bus) != ESP_OK) return ESP_ERR_TIMEOUT;
        DELAY(bus);
        SCL_LOW(bus);
    }

    /* ACK bit */
    SDA_HIGH(bus);
    DELAY(bus);
    if (scl_high(bus) != ESP_OK) return ESP_ERR_TIMEOUT;
    int nack = SDA_GET(bus);
    DELAY(bus);
    SCL_LOW(bus);

    return nack ? ESP_FAIL : ESP_OK;
}

static esp_err_t i2c_read_byte(soft_smbus_handle_t bus, uint8_t *data, bool ack)
{
    uint8_t res = 0;
    SDA_HIGH(bus);
    for (int i = 0; i < 8; i++) {
        DELAY(bus);
        if (scl_high(bus) != ESP_OK) return ESP_ERR_TIMEOUT;
        res = (res << 1) | (SDA_GET(bus) ? 1 : 0);
        DELAY(bus);
        SCL_LOW(bus);
    }

    gpio_set_level(bus->sda_io, ack ? 0 : 1);
    DELAY(bus);
    if (scl_high(bus) != ESP_OK) return ESP_ERR_TIMEOUT;
    DELAY(bus);
    SCL_LOW(bus);
    SDA_HIGH(bus);

    *data = res;
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * PEC: CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0, computed over
 * every byte of the message including the address bytes.
 * ------------------------------------------------------------------------- */
static uint8_t crc8(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* Write a byte and fold it into the running PEC. */
static esp_err_t tx(soft_smbus_handle_t bus, uint8_t b, uint8_t *crc)
{
    *crc = crc8(*crc, b);
    return i2c_write_byte(bus, b);
}

/* Read a byte and fold it into the running PEC. */
static esp_err_t rx(soft_smbus_handle_t bus, uint8_t *b, bool ack, uint8_t *crc)
{
    esp_err_t ret = i2c_read_byte(bus, b, ack);
    if (ret == ESP_OK) *crc = crc8(*crc, *b);
    return ret;
}

/* Write-side PEC: send the accumulated CRC as the last byte. */
static esp_err_t pec_send(soft_smbus_handle_t bus, uint8_t crc)
{
    return bus->pec ? i2c_write_byte(bus, crc) : ESP_OK;
}

/* Read-side PEC: read the slave's CRC byte (NACK) and compare. */
static esp_err_t pec_check(soft_smbus_handle_t bus, uint8_t crc)
{
    if (!bus->pec) return ESP_OK;
    uint8_t pec;
    esp_err_t ret = i2c_read_byte(bus, &pec, false);
    if (ret != ESP_OK) return ret;
    if (pec != crc) {
        ESP_LOGW(TAG, "PEC mismatch: got 0x%02X, expected 0x%02X", pec, crc);
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

/* START + (addr|W) + cmd. Caller must issue STOP. */
static esp_err_t smbus_begin(soft_smbus_handle_t bus, uint8_t addr, uint8_t cmd, uint8_t *crc)
{
    *crc = 0;
    esp_err_t ret = i2c_start(bus);
    if (ret == ESP_OK) ret = tx(bus, addr << 1, crc);
    if (ret == ESP_OK) ret = tx(bus, cmd, crc);
    return ret;
}

/* Repeated START + (addr|R). */
static esp_err_t smbus_restart_read(soft_smbus_handle_t bus, uint8_t addr, uint8_t *crc)
{
    esp_err_t ret = i2c_start(bus);
    if (ret == ESP_OK) ret = tx(bus, (addr << 1) | 1, crc);
    return ret;
}

/*
 * Generic SMBus transfer:
 *   S addr|W cmd [wr...] [PEC]                         (write)
 *   S addr|W cmd Sr addr|R rd... [PEC] (NACK last)     (read)
 */
static esp_err_t smbus_xfer(soft_smbus_handle_t bus, uint8_t addr, uint8_t cmd,
                            const uint8_t *wr, size_t wr_len,
                            uint8_t *rd, size_t rd_len)
{
    if (bus == NULL || addr > 0x7F) return ESP_ERR_INVALID_ARG;

    uint8_t crc;
    esp_err_t ret = smbus_begin(bus, addr, cmd, &crc);
    for (size_t i = 0; ret == ESP_OK && i < wr_len; i++) {
        ret = tx(bus, wr[i], &crc);
    }
    if (ret == ESP_OK && rd_len) {
        ret = smbus_restart_read(bus, addr, &crc);
        for (size_t i = 0; ret == ESP_OK && i < rd_len; i++) {
            ret = rx(bus, &rd[i], (i + 1 < rd_len) || bus->pec, &crc);
        }
        if (ret == ESP_OK) ret = pec_check(bus, crc);
    } else if (ret == ESP_OK) {
        ret = pec_send(bus, crc);
    }
    esp_err_t stop_ret = i2c_stop(bus);
    return ret != ESP_OK ? ret : stop_ret;
}

esp_err_t soft_smbus_new_bus(const soft_smbus_config_t *config, soft_smbus_handle_t *out_handle)
{
    if (!config || !out_handle || config->clk_speed_hz == 0 ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->sda_io) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->scl_io)) {
        return ESP_ERR_INVALID_ARG;
    }

    soft_smbus_handle_t bus = malloc(sizeof(*bus));
    if (!bus) return ESP_ERR_NO_MEM;

    bus->sda_io = config->sda_io;
    bus->scl_io = config->scl_io;
    /* Round up so the bus never runs faster than requested. */
    uint32_t div = config->clk_speed_hz * 2;
    bus->half_period_us = (1000000 + div - 1) / div;
    bus->timeout_us = (int64_t)config->timeout_ms * 1000;
    bus->pec = config->pec_enable;

    /* Pre-load output latch high so configuring OD doesn't glitch the bus low. */
    gpio_set_level(bus->sda_io, 1);
    gpio_set_level(bus->scl_io, 1);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pin_bit_mask = (1ULL << bus->sda_io) | (1ULL << bus->scl_io),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        free(bus);
        return ret;
    }

    SDA_HIGH(bus);
    gpio_set_level(bus->scl_io, 1);

    *out_handle = bus;
    return ESP_OK;
}

esp_err_t soft_smbus_del_bus(soft_smbus_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    gpio_reset_pin(handle->sda_io);
    gpio_reset_pin(handle->scl_io);
    free(handle);
    return ESP_OK;
}

esp_err_t soft_smbus_set_pec(soft_smbus_handle_t handle, bool enable)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    handle->pec = enable;
    return ESP_OK;
}

esp_err_t soft_smbus_probe(soft_smbus_handle_t handle, uint8_t address)
{
    if (handle == NULL || address > 0x7F) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_start(handle);
    if (ret == ESP_OK) ret = i2c_write_byte(handle, (uint8_t)(address << 1));
    esp_err_t stop_ret = i2c_stop(handle);
    return ret != ESP_OK ? ret : stop_ret;
}

esp_err_t soft_smbus_read_byte(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    return smbus_xfer(handle, dev_addr, reg_addr, NULL, 0, data, 1);
}

esp_err_t soft_smbus_write_byte(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return smbus_xfer(handle, dev_addr, reg_addr, &data, 1, NULL, 0);
}

esp_err_t soft_smbus_read_word(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint16_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t ret = smbus_xfer(handle, dev_addr, reg_addr, NULL, 0, buf, 2);
    if (ret == ESP_OK) *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return ret;
}

esp_err_t soft_smbus_write_word(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data & 0xFF), (uint8_t)(data >> 8) };
    return smbus_xfer(handle, dev_addr, reg_addr, buf, 2, NULL, 0);
}

esp_err_t soft_smbus_write_block(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len)
{
    if (handle == NULL || dev_addr > 0x7F || (len && !data)) return ESP_ERR_INVALID_ARG;

    uint8_t crc;
    esp_err_t ret = smbus_begin(handle, dev_addr, reg_addr, &crc);
    if (ret == ESP_OK) ret = tx(handle, len, &crc);
    for (uint8_t i = 0; ret == ESP_OK && i < len; i++) {
        ret = tx(handle, data[i], &crc);
    }
    if (ret == ESP_OK) ret = pec_send(handle, crc);
    esp_err_t stop_ret = i2c_stop(handle);
    return ret != ESP_OK ? ret : stop_ret;
}

esp_err_t soft_smbus_read_block(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t max_len, uint8_t *out_len)
{
    if (handle == NULL || dev_addr > 0x7F || !out_len || (max_len && !data)) return ESP_ERR_INVALID_ARG;

    uint8_t len = 0, crc;
    esp_err_t ret = smbus_begin(handle, dev_addr, reg_addr, &crc);
    if (ret == ESP_OK) ret = smbus_restart_read(handle, dev_addr, &crc);
    if (ret == ESP_OK) ret = rx(handle, &len, true, &crc);

    /* Read every byte the slave announces (it expects that many clocks),
     * but only store what fits in the caller's buffer. */
    for (uint8_t i = 0; ret == ESP_OK && i < len; i++) {
        uint8_t b;
        ret = rx(handle, &b, ((uint8_t)(i + 1) < len) || handle->pec, &crc);
        if (ret == ESP_OK && i < max_len) data[i] = b;
    }
    if (ret == ESP_OK) ret = pec_check(handle, crc);

    esp_err_t stop_ret = i2c_stop(handle);
    if (ret == ESP_OK) ret = stop_ret;
    if (ret == ESP_OK) *out_len = (len > max_len) ? max_len : len;
    return ret;
}

esp_err_t soft_smbus_read_df_reg(soft_smbus_handle_t handle, uint8_t dev_addr, uint16_t df_addr, uint8_t *data, uint8_t max_len, uint8_t *out_len)
{
    if (!out_len || (max_len && !data)) return ESP_ERR_INVALID_ARG;

    const uint8_t addr_payload[2] = { (uint8_t)(df_addr & 0xFF), (uint8_t)(df_addr >> 8) };

    /* Step 1: write DF address to ManufacturerBlockAccess */
    esp_err_t ret = soft_smbus_write_block(handle, dev_addr, BQ_MAC_CMD, addr_payload, sizeof(addr_payload));
    if (ret != ESP_OK) return ret;

    /* Give the gauge time to process; yield instead of busy-waiting. */
    TickType_t ticks = pdMS_TO_TICKS(BQ_MAC_DELAY_MS);
    vTaskDelay(ticks ? ticks : 1);

    /* Step 2: response is [Len] [Addr L] [Addr H] [Data...] */
    uint8_t full_block[SMBUS_BLOCK_MAX + 2];
    uint8_t full_len;
    ret = soft_smbus_read_block(handle, dev_addr, BQ_MAC_CMD, full_block, sizeof(full_block), &full_len);
    if (ret != ESP_OK) return ret;
    if (full_len < 2) return ESP_ERR_INVALID_RESPONSE;

    uint8_t data_len = full_len - 2;
    *out_len = (data_len > max_len) ? max_len : data_len;
    memcpy(data, &full_block[2], *out_len);
    return ESP_OK;
}

esp_err_t soft_smbus_write_df_reg(soft_smbus_handle_t handle, uint8_t dev_addr, uint16_t df_addr, const uint8_t *data, uint8_t len)
{
    if (len > SMBUS_BLOCK_MAX - 2 || (len && !data)) return ESP_ERR_INVALID_ARG;

    uint8_t payload[SMBUS_BLOCK_MAX];
    payload[0] = (uint8_t)(df_addr & 0xFF);   /* little endian */
    payload[1] = (uint8_t)(df_addr >> 8);
    if (len) memcpy(&payload[2], data, len);

    return soft_smbus_write_block(handle, dev_addr, BQ_MAC_CMD, payload, len + 2);
}
