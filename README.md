# esp-idf-smbus-driver

A software (bit-banged) SMBus master for ESP-IDF, with helpers for TI BQ-series
fuel gauges (Data Flash access through `ManufacturerBlockAccess`, 0x44).

It runs on any two GPIOs, supports clock stretching with a timeout, and does not
use the hardware I2C peripheral.

## Features

- SMBus Quick probe, Read/Write Byte, Read/Write Word, Block Read/Write
- Data Flash read/write for BQ gauges (`soft_smbus_read_df_reg` / `soft_smbus_write_df_reg`)
- Optional PEC (Packet Error Checking, CRC-8) on all transfers except the probe
- Clock stretching with a configurable timeout
- Open-drain GPIOs: no direction switching in the bit loop

## Requirements

- ESP-IDF v6.x (tested build: v6.1.1, target `esp32`)
- External pull-up resistors on SDA and SCL (4.7 kΩ–10 kΩ to 3.3 V).
  The internal pull-ups are enabled but are too weak on their own.

## Project layout

```
components/soft_smbus/       Driver component
  include/soft_smbus.h       Public API
  soft_smbus.c               Implementation
main/esp-idf-smbus-driver.c  Example application
```

## Build and flash

```sh
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

## Example

The example in `main/` scans the bus, then reads a Smart Battery at address `0x0B`:
manufacturer and device name, one Data Flash register, and voltage, current,
temperature and state of charge every second.

Default pins are SDA = GPIO21, SCL = GPIO22. Change them at the top of
`main/esp-idf-smbus-driver.c`.

```c
#include "soft_smbus.h"

soft_smbus_handle_t smbus_handle;

void app_main(void)
{
    const soft_smbus_config_t cfg = {
        .sda_io = GPIO_NUM_21,
        .scl_io = GPIO_NUM_22,
        .clk_speed_hz = 100000,
        .timeout_ms = 35,
        .pec_enable = true,
    };
    ESP_ERROR_CHECK(soft_smbus_new_bus(&cfg, &smbus_handle));

    uint16_t mv;
    if (soft_smbus_read_word(smbus_handle, 0x0B, 0x09, &mv) == ESP_OK) {
        printf("Voltage: %u mV\n", mv);
    }
}
```

## PEC (Packet Error Checking)

When `pec_enable` is true, the driver adds an SMBus CRC-8 byte (polynomial
0x07) to every write and checks it on every read. The CRC covers the address
bytes, the command and all data. A mismatch returns `ESP_ERR_INVALID_CRC`.

You can switch PEC on or off at runtime:

```c
soft_smbus_set_pec(smbus_handle, false);
```

Only enable PEC if the device supports it. A device without PEC will not send
the extra byte on reads (the check fails), and may reject the extra byte on
writes. `soft_smbus_probe` never uses PEC.

## Using the driver in another project

Copy `components/soft_smbus` into your project's `components/` folder and add it
to the component that uses it:

```cmake
idf_component_register(SRCS "app.c"
                       PRIV_REQUIRES soft_smbus)
```

## API

| Function | SMBus protocol |
|---|---|
| `soft_smbus_new_bus` / `soft_smbus_del_bus` | Create / free a bus |
| `soft_smbus_set_pec` | Turn PEC on or off |
| `soft_smbus_probe` | Quick command (address ACK check) |
| `soft_smbus_read_byte` / `soft_smbus_write_byte` | Read / Write Byte |
| `soft_smbus_read_word` / `soft_smbus_write_word` | Read / Write Word (little-endian) |
| `soft_smbus_read_block` / `soft_smbus_write_block` | Block Read / Write (max 32 bytes) |
| `soft_smbus_read_df_reg` / `soft_smbus_write_df_reg` | BQ Data Flash via 0x44 |

All functions return `esp_err_t`:

- `ESP_OK`: success
- `ESP_FAIL`: the device did not ACK (NACK)
- `ESP_ERR_TIMEOUT`: SCL was held low longer than `timeout_ms`
- `ESP_ERR_INVALID_CRC`: PEC mismatch
- `ESP_ERR_INVALID_ARG`: bad handle, address or buffer
- `ESP_ERR_INVALID_RESPONSE`: Data Flash reply too short

## Limitations

- Not thread-safe: if several tasks share one bus, protect it with your own mutex.
- The real clock is a little slower than `clk_speed_hz` because of GPIO call
  overhead. This is within SMBus limits (10–100 kHz).
- The driver does not recover a stuck bus (it does not clock out a slave holding SDA low).
