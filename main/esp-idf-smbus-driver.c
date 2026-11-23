/*
 * Example: reading a Smart Battery / TI BQ fuel gauge over the software SMBus.
 *
 * Wiring (change the defines below to match your board):
 *   SDA -> GPIO21, SCL -> GPIO22, plus external 4.7k–10k pull-ups to 3.3 V.
 *   The internal pull-ups are enabled but are too weak for reliable use alone.
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soft_smbus.h"

#define SMBUS_SDA_IO        GPIO_NUM_21
#define SMBUS_SCL_IO        GPIO_NUM_22
#define SMBUS_CLK_HZ        100000      /* SMBus max is 100 kHz */
#define SMBUS_TIMEOUT_MS    35          /* SMBus clock-low timeout (tTIMEOUT) */

#define BATTERY_ADDR        0x0B        /* Standard Smart Battery address */

/* Smart Battery Data Specification (SBS) commands */
#define SBS_TEMPERATURE     0x08        /* 0.1 K   */
#define SBS_VOLTAGE         0x09        /* mV      */
#define SBS_CURRENT         0x0A        /* mA, signed */
#define SBS_RSOC            0x0D        /* %       */
#define SBS_MANUFACTURER    0x20        /* block, string */
#define SBS_DEVICE_NAME     0x21        /* block, string */

/* Example Data Flash address — look up the real one in your gauge's TRM. */
#define DF_EXAMPLE_ADDR     0x4FC4

static const char *TAG = "smbus_example";

/* Global handle declared in soft_smbus.h */
soft_smbus_handle_t smbus_handle = NULL;

static void scan_bus(void)
{
    ESP_LOGI(TAG, "Scanning bus...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (soft_smbus_probe(smbus_handle, addr) == ESP_OK) {
            ESP_LOGI(TAG, "  device found at 0x%02X", addr);
        }
    }
}

static void read_block_string(uint8_t cmd, const char *label)
{
    uint8_t buf[33];
    uint8_t len = 0;
    if (soft_smbus_read_block(smbus_handle, BATTERY_ADDR, cmd, buf, sizeof(buf) - 1, &len) == ESP_OK) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "%s: %s", label, (char *)buf);
    } else {
        ESP_LOGW(TAG, "Failed to read %s", label);
    }
}

static void read_data_flash(void)
{
    uint8_t data[32];
    uint8_t len = 0;
    esp_err_t ret = soft_smbus_read_df_reg(smbus_handle, BATTERY_ADDR, DF_EXAMPLE_ADDR,
                                           data, sizeof(data), &len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Data Flash read failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Data Flash @0x%04X (%u bytes):", DF_EXAMPLE_ADDR, len);
    ESP_LOG_BUFFER_HEX(TAG, data, len);
}

void app_main(void)
{
    const soft_smbus_config_t cfg = {
        .sda_io = SMBUS_SDA_IO,
        .scl_io = SMBUS_SCL_IO,
        .clk_speed_hz = SMBUS_CLK_HZ,
        .timeout_ms = SMBUS_TIMEOUT_MS,
    };
    ESP_ERROR_CHECK(soft_smbus_new_bus(&cfg, &smbus_handle));

    scan_bus();

    if (soft_smbus_probe(smbus_handle, BATTERY_ADDR) != ESP_OK) {
        ESP_LOGE(TAG, "No battery at 0x%02X, check wiring and pull-ups", BATTERY_ADDR);
        return;
    }

    read_block_string(SBS_MANUFACTURER, "Manufacturer");
    read_block_string(SBS_DEVICE_NAME, "Device");
    read_data_flash();

    while (1) {
        uint16_t voltage, current, temp, rsoc;
        if (soft_smbus_read_word(smbus_handle, BATTERY_ADDR, SBS_VOLTAGE, &voltage) == ESP_OK &&
            soft_smbus_read_word(smbus_handle, BATTERY_ADDR, SBS_CURRENT, &current) == ESP_OK &&
            soft_smbus_read_word(smbus_handle, BATTERY_ADDR, SBS_TEMPERATURE, &temp) == ESP_OK &&
            soft_smbus_read_word(smbus_handle, BATTERY_ADDR, SBS_RSOC, &rsoc) == ESP_OK) {
            ESP_LOGI(TAG, "V=%u mV  I=%d mA  T=%.1f C  SOC=%u %%",
                     voltage, (int16_t)current, temp / 10.0 - 273.15, rsoc);
        } else {
            ESP_LOGW(TAG, "Battery read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
