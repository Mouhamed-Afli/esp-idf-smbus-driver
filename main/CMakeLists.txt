idf_component_register(SRCS "esp-idf-smbus-driver.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES soft_smbus freertos log)
