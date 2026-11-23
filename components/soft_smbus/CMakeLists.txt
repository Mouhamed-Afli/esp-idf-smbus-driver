idf_component_register(SRCS "soft_smbus.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_timer esp_rom freertos log)
