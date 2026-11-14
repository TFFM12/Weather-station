#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *ESP = "ESP32-S3 (N16R8)";     
static const char *TAG1 = "REED_SWITCH";
static const char *TAG = "AS5600";
static const char *TAG_BOT = "BOTAO";

/**************************************************************************************************
 *         Parte do Defines e inicialização dos modulos, protocolos e interrupções 
 **************************************************************************************************/

/**
* @brief Parte relacionada ao AS5600 - Sentido do Vento
*/

// Configuração dos Pinos I2C no ESP32-S3
#define I2C_MASTER_SCL_IO           9       // GPIO para SCL
#define I2C_MASTER_SDA_IO           8       // GPIO para SDA
#define I2C_MASTER_NUM              I2C_NUM_0 // Porta I2C
#define I2C_MASTER_FREQ_HZ          400000  // Frequência I2C (400kHz)

// Endereço e Registos do AS5600
#define AS5600_ADDR                 0x36
#define AS5600_RAW_ANGLE_REGISTER   0x0C    // Registo de ângulo bruto (12 bits: 0-4095)
#define AS5600_STATUS_REGISTER      0x0B    // Registo de estado do íman

// Variáveis globais de I2C
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


/**
* @brief Parte relacionada ao Reed Switch - Velocidade do vento
*/

// Configuração do pino do Reed Switch
#define REED_SWITCH_GPIO        GPIO_NUM_4  
#define DEBOUNCE_TIME_MS        50          // Tempo para ignorar o ruído do contacto mecânico

// Fila para passar eventos de interrupção da ISR para a tarefa
static QueueHandle_t gpio_evt_queue = NULL;

// Configuração do botão de arranque
#define START_BOT       GPIO_NUM_5
volatile int estado_sistema = 0;


/**
 * @brief Interrupção (ISR) executada de cada vez que o pino deteta uma alteração de estado
 */

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // Envia o número do pino para a fila sem bloquear a interrupção
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**************************************************************************************************
 *                                       Contadores (tempo)     
 **************************************************************************************************/

volatile int contador_5s = 0;

void tarefa_alternar_estado(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));      // Espera 5 segundos sem bloquear o sistema
        contador_5s = !contador_5s; // Alterna entre 0 e 1
    }
}

uint32_t rotacoes = 0;

/**************************************************************************************************
 *       Função que vai tratar do Reed Switch (Velocidade do vento) e do Botão de arranque
 **************************************************************************************************/

static void gpio_control_task(void* arg)
{
    uint32_t io_num;
    int64_t ultimo_tempo_reed_ms = 0;
    int64_t ultimo_tempo_bot_ms = 0;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t tempo_atual_ms = esp_timer_get_time() / 1000;

            // Tratamento do Reed Switch
            if (io_num == REED_SWITCH_GPIO) {
                if ((tempo_atual_ms - ultimo_tempo_reed_ms) > DEBOUNCE_TIME_MS) {
                    if (gpio_get_level(io_num) == 0) {
                        rotacoes++;
                        ultimo_tempo_reed_ms = tempo_atual_ms;
                    }
                }
            } 
            // Tratamento do Botão (Toggle / Alternar estado)
            else if (io_num == START_BOT) {
                if ((tempo_atual_ms - ultimo_tempo_bot_ms) > 200) { // Debounce de 200ms para o botão
                    if (gpio_get_level(io_num) == 0) {
                        estado_sistema = !estado_sistema; // Inverte entre 0 e 1
                        ESP_LOGI(TAG_BOT, "Botão pressionado! Novo estado_sistema: %d", estado_sistema);
                        ultimo_tempo_bot_ms = tempo_atual_ms;
                    }
                }
            }
        }
    }
}

/**************************************************************************************************
 *                              Configurações dos pinos GPIOs
 **************************************************************************************************/
static void gpio_init(void)
{
    // Configuração dos dois pinos (Reed Switch e Botão) com interrupção na descida (NEGEDGE)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << REED_SWITCH_GPIO) | (1ULL << START_BOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_install_isr_service(0);
    
    // Regista os handlers de interrupção para ambos os pinos
    gpio_isr_handler_add(REED_SWITCH_GPIO, gpio_isr_handler, (void*) REED_SWITCH_GPIO);
    gpio_isr_handler_add(START_BOT, gpio_isr_handler, (void*) START_BOT);
}

/**************************************************************************************************
 *                                        Parte do AS5600
 **************************************************************************************************/

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

/**************************************************************************************************
 *                                            MAIN
 **************************************************************************************************/

void app_main(void)
{
    ESP_LOGI(ESP, "Inicializar");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(ESP, "Inicializado com sucesso!");
    
    gpio_init(); // Faz o arranque dos pinos GPIO e interrupções

    xTaskCreate(tarefa_alternar_estado, "task_5s", 2048, NULL, 5, NULL);
    // Alterado de reed_switch_task para gpio_control_task
    xTaskCreate(gpio_control_task, "gpio_control_task", 2048, NULL, 10, NULL);

    while (1) {
        if (estado_sistema == 0) {
            // Cede tempo à CPU quando o sistema está desligado no botão
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            if (contador_5s == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                uint8_t status = as5600_read_status();
                uint16_t raw_angle = as5600_read_raw_angle();

                float degrees = (raw_angle * 360.0f) / 4096.0f;

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
                    ESP_LOGI(TAG, "Ângulo Bruto: %4d | Ângulo: %6.2f°", raw_angle, degrees);
                }
                ESP_LOGI(TAG1, "Nº Rotações %lu", rotacoes);
                contador_5s = !contador_5s;
}}}}