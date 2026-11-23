#ifndef SOFT_SMBUS_H
#define SOFT_SMBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration du bus SMBus logiciel
 */
typedef struct {
    gpio_num_t sda_io;      /*!< Numéro GPIO pour SDA */
    gpio_num_t scl_io;      /*!< Numéro GPIO pour SCL */
    uint32_t clk_speed_hz;  /*!< Vitesse d'horloge (ex: 100000 pour 100kHz) */
    uint32_t timeout_ms;    /*!< Timeout pour le clock stretching en ms */
    bool pec_enable;        /*!< Active le PEC (CRC-8) sur toutes les transactions */
} soft_smbus_config_t;

typedef struct soft_smbus_obj_t* soft_smbus_handle_t;

// Global SMBus handle for the application
extern soft_smbus_handle_t smbus_handle;

/**
 * @brief Initialise un nouveau bus SMBus logiciel
 * 
 * @param config Configuration du bus
 * @param out_handle Handle du bus créé
 * @return esp_err_t ESP_OK en cas de succès
 */
esp_err_t soft_smbus_new_bus(const soft_smbus_config_t *config, soft_smbus_handle_t *out_handle);

/**
 * @brief Supprime un bus SMBus logiciel
 * 
 * @param handle Handle du bus
 * @return esp_err_t ESP_OK en cas de succès
 */
esp_err_t soft_smbus_del_bus(soft_smbus_handle_t handle);

/**
 * @brief Active ou désactive le PEC (Packet Error Checking) à l'exécution
 *
 * Avec le PEC actif, un octet CRC-8 (poly 0x07) est ajouté aux écritures et
 * vérifié sur les lectures. Une erreur de CRC renvoie ESP_ERR_INVALID_CRC.
 * Le probe (Quick Command) n'utilise jamais le PEC.
 *
 * @param handle Handle du bus
 * @param enable true pour activer le PEC
 * @return esp_err_t ESP_OK en cas de succès
 */
esp_err_t soft_smbus_set_pec(soft_smbus_handle_t handle, bool enable);

/**
 * @brief Probing d'un périphérique SMBus
 * @param handle Handle du bus
 * @param address Adresse du périphérique
 * @return esp_err_t ESP_OK en cas de succès
 */
esp_err_t soft_smbus_probe(soft_smbus_handle_t handle, uint8_t address);

/**
 * @brief Lecture d'un mot SMBus (Word Read)
 */
esp_err_t soft_smbus_read_word(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint16_t *data);

/**
 * @brief Écriture d'un mot SMBus (Word Write)
 */
esp_err_t soft_smbus_write_word(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint16_t data);

/**
 * @brief Lecture d'un octet SMBus (Byte Read)
 */
esp_err_t soft_smbus_read_byte(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Écriture d'un octet SMBus (Byte Write)
 */
esp_err_t soft_smbus_write_byte(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Lecture d'un bloc SMBus (Block Read)
 * 
 * @param handle Handle du bus
 * @param dev_addr Adresse de l'esclave (7 bits)
 * @param reg_addr Adresse du registre
 * @param data Buffer pour stocker les données lues
 * @param max_len Taille maximale du buffer
 * @param out_len Nombre d'octets réellement lus
 * @return esp_err_t ESP_OK en cas de succès
 */
esp_err_t soft_smbus_read_block(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t max_len, uint8_t *out_len);

/**
 * @brief Écriture d'un bloc SMBus (Block Write)
 */
esp_err_t soft_smbus_write_block(soft_smbus_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len);

/**
 * @brief Lecture d'un registre de la Data Flash (via ManufacturerBlockAccess 0x44)
 * 
 * @param handle Handle du bus
 * @param dev_addr Adresse de l'esclave
 * @param df_addr Adresse de la Data Flash (ex: 0x4FC4)
 * @param data Buffer pour stocker les données lues
 * @param max_len Taille maximale à lire
 * @param out_len Nombre d'octets lus
 * @return esp_err_t 
 */
esp_err_t soft_smbus_read_df_reg(soft_smbus_handle_t handle, uint8_t dev_addr, uint16_t df_addr, uint8_t *data, uint8_t max_len, uint8_t *out_len);

/**
 * @brief Écriture d'un registre de la Data Flash (via ManufacturerBlockAccess 0x44)
 *
 * @param len Nombre d'octets à écrire (max 30)
 */
esp_err_t soft_smbus_write_df_reg(soft_smbus_handle_t handle, uint8_t dev_addr, uint16_t df_addr, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // SOFT_SMBUS_H
