#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

// Configuração dos Pinos I2C no ESP32-S3
#define I2C_MASTER_SCL_IO           9       /*!< GPIO para SCL */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO para SDA */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< Porta I2C */
#define I2C_MASTER_FREQ_HZ          400000  /*!< Frequência I2C (400kHz) */

// Endereço e Registos do AS5600
#define AS5600_ADDR                 0x36
#define AS5600_RAW_ANGLE_REGISTER   0x0C    /*!< Registo de ângulo bruto (12 bits: 0-4095) */
#define AS5600_STATUS_REGISTER      0x0B    /*!< Registo de estado do íman */

// Variáveis globais de I2C
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

/**
 * @brief Inicializa o barramento e o dispositivo I2C
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    return ESP_OK;
}

/**
 * @brief Lê o valor bruto do ângulo (0 - 4095)
 */
static uint16_t as5600_read_raw_angle(void)
{
    uint8_t reg = AS5600_RAW_ANGLE_REGISTER;
    uint8_t data[2] = {0};

    // Escreve o endereço do registo e lê 2 bytes (MSB e LSB)
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro na leitura I2C: %s", esp_err_to_name(err));
        return 0;
    }

    // O AS5600 devolve um valor de 12-bit (data[0] tem os 4 bits mais significativos)
    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    return raw_angle & 0x0FFF;
}

/**
 * @brief Verifica se o íman está posicionado corretamente
 */
static uint8_t as5600_read_status(void)
{
    uint8_t reg = AS5600_STATUS_REGISTER;
    uint8_t status = 0;

    i2c_master_transmit_receive(dev_handle, &reg, 1, &status, 1, -1);
    return status;
}

void app_main(void)
{
    ESP_LOGI(TAG, "A inicializar o barramento I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Inicializado com sucesso!");

    while (1) {
        uint8_t status = as5600_read_status();
        uint16_t raw_angle = as5600_read_raw_angle();

        // Converte o valor de 12 bits (0-4095) para graus (0.0 - 360.0)
        float degrees = (raw_angle * 360.0f) / 4096.0f;

        // Verificação do estado do íman (Bits: 5 = MD (Magnet Detected), 4 = ML (Too Low), 3 = MH (Too High))
        bool magnet_detected = (status & (1 << 5)) != 0;
        bool magnet_too_weak = (status & (1 << 4)) != 0;
        bool magnet_too_strong = (status & (1 << 3)) != 0;

        if (!magnet_detected) {
            ESP_LOGW(TAG, "Aviso: Íman não detetado!");
        } else if (magnet_too_weak) {
            ESP_LOGW(TAG, "Aviso: Íman demasiado longe/fraco!");
        } else if (magnet_too_strong) {
            ESP_LOGW(TAG, "Aviso: Íman demasiado perto/forte!");
        } else {
            // Imprime no Monitor Serial
            ESP_LOGI(TAG, "Ângulo Bruto: %4d | Ângulo: %6.2f°", raw_angle, degrees);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Atualiza a cada 200 ms
    }
}