#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG_ESP  = "ESP32-S3 (N16R8)";     
static const char *TAG_REED = "REED_SWITCH";
static const char *TAG_PLUV = "PLUVIOMETRO";
static const char *TAG_SHT  = "SHT31";
static const char *TAG_AS   = "AS5600";
static const char *TAG_BOT  = "BOTAO";

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
#define AS5600_ZPOS_REGISTER        0x01    // Registo de posição Zero (MSB: 0x01, LSB: 0x02)
#define AS5600_ANGLE_REGISTER       0x0E    // Registo de ângulo já com offset ZPOS aplicado!

// Endereço e Comandos do SHT31-D (Temperatura e Humidade)
#define SHT31_SENSOR_ADDR           0x44    // Endereço padrão (0x45 se o pino ADDR estiver a VCC)
#define SHT31_CMD_MEAS_HIGH_MSB     0x24    // Single-shot, alta repetibilidade, sem clock stretching
#define SHT31_CMD_MEAS_HIGH_LSB     0x00

// Variáveis globais de I2C - Barramente compartilhado por dois sensores
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t as5600_dev_handle;
i2c_master_dev_handle_t sht31_dev_handle;

/**
* @brief Parte relacionada ao Reed Switch - Velocidade do vento
*/

// Configuração do pino do Reed Switch
#define REED_SWITCH_GPIO        GPIO_NUM_4

// Configuração do pino do pluviometro (Reed_switch)
#define PLUV_GPIO               GPIO_NUM_6

// Configuração do DEBOUNCE
#define DEBOUNCE_TIME_MS        70          // Tempo para ignorar o ruído do contacto mecânico
#define DEBOUNCE_TIME_MS_REED   10          // Tempo para ignorar o ruído do contacto mecânico

// Fila para passar eventos de interrupção da ISR para a tarefa
static QueueHandle_t gpio_evt_queue = NULL;

// Tempo para fazer dois ciclos com o pluviometro

volatile float media_tempo_pluv = 0.0f;
volatile float ultimo_tempo_2_ciclos_ms = 0.0f;
volatile float tempo_2ciclos = 0.0f;

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

volatile int contador_10s = 0;

void tarefa_alternar_estado(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));      // Espera 10 segundos sem bloquear o sistema
        contador_10s = !contador_10s; // Alterna entre 0 e 1
    }
}

atomic_uint_fast32_t rotacoes = 0;

/**************************************************************************************************
 *                                   Parte do Barramento I2C     
 **************************************************************************************************/

/**
 * @brief Inicializa o barramento I2C e regista o AS5600 e o SHT31-D
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

    // Dispositivo 1: AS5600
    i2c_device_config_t as5600_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &as5600_cfg, &as5600_dev_handle));

    // Dispositivo 2: SHT31-D
    i2c_device_config_t sht31_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT31_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sht31_cfg, &sht31_dev_handle));

    return ESP_OK;
}

/**************************************************************************************************
 *       Função que vai tratar do Reed Switch (Velocidade do vento) e do Botão de arranque
 **************************************************************************************************/

 static void gpio_control_task(void* arg)
{
    uint32_t io_num;
    int64_t ultimo_tempo_reed_ms = 0;
    int64_t ultimo_tempo_bot_ms = 0;

    // Variáveis de estado e temporização para o pluviómetro (2 ciclos)
    int64_t ultimo_tempo_reed2_us = 0;
    int64_t t_inicio_reed2_us = 0;
    int estado_anterior_reed2 = 1; // 1 = aberto (pull-up), 0 = fechado (GND)
    int ciclos_reed2 = 0;
    bool em_contagem_reed2 = false;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t tempo_atual_us = esp_timer_get_time();
            int64_t tempo_atual_ms = tempo_atual_us / 1000;

            // 1. Tratamento do Reed Switch (Rotação)
            if (io_num == REED_SWITCH_GPIO) {
                if ((tempo_atual_ms - ultimo_tempo_reed_ms) > DEBOUNCE_TIME_MS_REED) {
                    if (gpio_get_level(io_num) == 0) {
                        rotacoes++;
                        ultimo_tempo_reed_ms = tempo_atual_ms;
                    }
                }
            } 
            // 2. Tratamento do Pluviómetro (Tempo para ligar e desligar 2 vezes)
            else if (io_num == PLUV_GPIO) {
                if (estado_sistema == 1 && (tempo_atual_us - ultimo_tempo_reed2_us) > (DEBOUNCE_TIME_MS * 1000)) {
                    int nivel_atual = gpio_get_level(PLUV_GPIO);

                    // Só processa se houve mudança real de estado (evita falsos disparos de bounce)
                    if (nivel_atual != estado_anterior_reed2) {
                        estado_anterior_reed2 = nivel_atual;
                        ultimo_tempo_reed2_us = tempo_atual_us;

                        if (nivel_atual == 0) {
                            // Contacto fechou ("ligar")
                            if (!em_contagem_reed2) {
                                em_contagem_reed2 = true;
                                ciclos_reed2 = 0;
                                t_inicio_reed2_us = tempo_atual_us;
                                //ESP_LOGI(TAG_PLUV, "1.º Ligar detetado — cronómetro iniciado...");
                            }} else {
                            // Contacto abriu ("desligar")
                            if (em_contagem_reed2) {
                                ciclos_reed2++;
                                if (ciclos_reed2 == 1) {
                                    //ESP_LOGI(TAG_PLUV, "1.º Desligar (1/2 ciclos concluído)");
                                } else if (ciclos_reed2 == 2) {
                                    int64_t duracao_us = tempo_atual_us - t_inicio_reed2_us;
                                    ultimo_tempo_2_ciclos_ms = duracao_us / 1000.0f;

                                    //ESP_LOGI(TAG_PLUV, "2.º Desligar (2/2 ciclos concluídos) | Tempo: %.2f ms (%.3f s)",
                                    //         ultimo_tempo_2_ciclos_ms, duracao_us / 1000000.0f);
                                    media_tempo_pluv = ultimo_tempo_2_ciclos_ms / 2.0f;
                                    tempo_2ciclos = ultimo_tempo_2_ciclos_ms;
                                    // Reinicia para a próxima medição de 2 ciclos
                                    em_contagem_reed2 = false;
                                    ciclos_reed2 = 0;
                                    ultimo_tempo_2_ciclos_ms = 0;
                                }}}}}}      
            // 3. Tratamento do Botão (Toggle / Alternar estado)
            else if (io_num == START_BOT) {
                if ((tempo_atual_ms - ultimo_tempo_bot_ms) > 200) { // Debounce de 200ms para o botão
                    if (gpio_get_level(io_num) == 0) {
                        estado_sistema = !estado_sistema; // Inverte entre 0 e 1
                        if (estado_sistema == 0) {
                            em_contagem_reed2 = false;
                            ciclos_reed2 = 0;
                            estado_anterior_reed2 = 1;
                        }
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
    gpio_config_t io_conf_negedge = {
        .pin_bit_mask = (1ULL << REED_SWITCH_GPIO) | (1ULL << START_BOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf_negedge);

    gpio_config_t io_conf_anyedge = {
        .pin_bit_mask = (1ULL << PLUV_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf_anyedge);

    gpio_evt_queue = xQueueCreate(20, sizeof(uint32_t));

    gpio_install_isr_service(0);
    
    // Regista os handlers de interrupção para ambos os pinos
    gpio_isr_handler_add(REED_SWITCH_GPIO, gpio_isr_handler, (void*) REED_SWITCH_GPIO);
    gpio_isr_handler_add(START_BOT, gpio_isr_handler, (void*) START_BOT);
    gpio_isr_handler_add(PLUV_GPIO, gpio_isr_handler, (void*) PLUV_GPIO);
}

/**************************************************************************************************
 *                                        Parte do AS5600
 **************************************************************************************************/

 /**
 * @brief Lê o valor bruto do ângulo (0 - 4095) do registo 0x0C
 */
static uint16_t as5600_read_raw_angle(void)
{
    uint8_t reg = AS5600_RAW_ANGLE_REGISTER;
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(as5600_dev_handle, &reg, 1, data, 2, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AS, "Erro na leitura I2C: %s", esp_err_to_name(err));
        return 0;
    }

    return (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF;
}

/**
 * @brief Define a posição atual do íman como 0° (Norte) diretamente no AS5600
 */
static esp_err_t as5600_set_zero_position(void)
{
    uint16_t raw = as5600_read_raw_angle(); // Lê o ângulo bruto atual (0x0C)
    uint8_t cmd[3] = {
        AS5600_ZPOS_REGISTER,
        (uint8_t)((raw >> 8) & 0x0F),       // ZPOS MSB (4 bits)
        (uint8_t)(raw & 0xFF)               // ZPOS LSB (8 bits)
    };
    return i2c_master_transmit(as5600_dev_handle, cmd, sizeof(cmd), 100);
}

/**
 * @brief Lê o ângulo já calibrado com o zero (0 - 4095) do registo 0x0E
 */
static uint16_t as5600_read_angle(void)
{
    uint8_t reg = AS5600_ANGLE_REGISTER; // <-- Corrigido para 0x0E!
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(as5600_dev_handle, &reg, 1, data, 2, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AS, "Erro na leitura I2C: %s", esp_err_to_name(err));
        return 0;
    }

    return (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF;
}
/**
 * @brief Verifica se o íman está posicionado corretamente
 */
static uint8_t as5600_read_status(void)
{
    uint8_t reg = AS5600_STATUS_REGISTER;
    uint8_t status = 0;

    i2c_master_transmit_receive(as5600_dev_handle, &reg, 1, &status, 1, 100);
    return status;
}

/**************************************************************************************************
 *                                        Parte do SHT31-D
 **************************************************************************************************/

/**
 * @brief Calcula o CRC-8 (polinómio 0x31, valor inicial 0xFF) exigido pelo SHT31-D
 */
static uint8_t sht31_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Dispara uma medição no SHT31-D e devolve Temperatura (°C) e Humidade Relativa (%)
 */

static esp_err_t sht31_read_temp_hum(float *temperature, float *humidity)
{
    uint8_t cmd[2] = { SHT31_CMD_MEAS_HIGH_MSB, SHT31_CMD_MEAS_HIGH_LSB };
    uint8_t data[6] = { 0 };

    // 1. Envia comando de início de conversão
    esp_err_t err = i2c_master_transmit(sht31_dev_handle, cmd, sizeof(cmd), 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT, "Erro ao enviar comando I2C: %s", esp_err_to_name(err));
        return err;
    }

    // 2. Aguarda o tempo máximo de medição (15.5 ms em alta repetibilidade)
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. Lê 6 bytes: [Temp MSB, Temp LSB, Temp CRC, Hum MSB, Hum LSB, Hum CRC]
    err = i2c_master_receive(sht31_dev_handle, data, sizeof(data), 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT, "Erro ao ler dados I2C: %s", esp_err_to_name(err));
        return err;
    }

    // 4. Verifica integridade dos dados via CRC-8
    if (sht31_crc8(&data[0], 2) != data[2] || sht31_crc8(&data[3], 2) != data[5]) {
        ESP_LOGE(TAG_SHT, "Erro de CRC nos dados recebidos do SHT31-D!");
        return ESP_ERR_INVALID_CRC;
    }

    // 5. Converte valores brutos de 16 bits segundo o datasheet Sensirion
    uint16_t raw_temp = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_hum  = ((uint16_t)data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *humidity    = 100.0f * ((float)raw_hum / 65535.0f);

    return ESP_OK;
}

/**************************************************************************************************
 *                                            MAIN
 **************************************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG_ESP, "Inicializar");
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(50));
    as5600_set_zero_position(); // Grava a posição inicial como 0° (Norte)
    ESP_LOGI(TAG_ESP, "Inicializado com sucesso!");
    
    gpio_init(); // Faz o arranque dos pinos GPIO e interrupções

    xTaskCreate(tarefa_alternar_estado, "task_5s", 2048, NULL, 5, NULL);
    // Alterado de reed_switch_task para gpio_control_task
    xTaskCreate(gpio_control_task, "gpio_control_task", 3072, NULL, 10, NULL);

    while (1) {
        if (estado_sistema == 0) {
            // Cede tempo à CPU quando o sistema está desligado no botão
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            if (contador_10s == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                
                uint8_t status = as5600_read_status();
                uint16_t angle = as5600_read_angle();
                float degrees = (angle * 360.0f) / 4096.0f;

                static const char *rosa[8] = {"N", "NE", "E", "SE", "S", "SO", "O", "NO"};
                int idx = ((int)((degrees + 22.5f) / 45.0f)) % 8;

                bool magnet_detected = (status & (1 << 5)) != 0;
                bool magnet_too_weak = (status & (1 << 4)) != 0;
                bool magnet_too_strong = (status & (1 << 3)) != 0;

                if (!magnet_detected) {
                    ESP_LOGW(TAG_AS, "Aviso: Íman não detetado!");
                } else if (magnet_too_weak) {
                    ESP_LOGW(TAG_AS, "Aviso: Íman demasiado longe/fraco!");
                } else if (magnet_too_strong) {
                    ESP_LOGW(TAG_AS, "Aviso: Íman demasiado perto/forte!");
                } else {
                    ESP_LOGI(TAG_AS, "Ângulo: %6.2f° | Direção: %s", degrees, rosa[idx]);
                }

                // Lê o valor atual de 'rotacoes' e coloca-o a 0 no mesmo instante indivisível
                uint32_t rotacoes_lidas = atomic_exchange(&rotacoes, 0);

                ESP_LOGI(TAG_REED, "Nº Rotações %lu", rotacoes_lidas);
                float f = rotacoes_lidas / 10.0f;
                ESP_LOGI(TAG_REED, "Frequência %0.3f Hz", f);
                float v = 0.975f * f;
                ESP_LOGI(TAG_REED, "Velocidade %0.3f m/s", v);

                float temperatura = 0.0f;
                float humidade = 0.0f;
                if (sht31_read_temp_hum(&temperatura, &humidade) == ESP_OK) {
                    ESP_LOGI(TAG_SHT, "Temperatura: %.2f °C | Humidade: %.2f %%", temperatura, humidade);
                }

                // --- CALIBRAÇÃO DO PLUVIÓMETRO ---
                // Diâmetro superior do funil = 52 mm -> Raio = 26 mm (2.6 cm)
                // Área de captação = PI * (2.6 * 2.6) = 21.237 cm² (ou 2123.72 mm²)
                // Volume de 1 lado da gangorra = 1.7 ml (1700 mm³)
                //
                // Chuva por tombada = 1700 mm³ / 2123.72 mm² = 0.8005 mm

                    //altura do cone = 50,04 mm, diametro em cima =  52mm, diametro em baixo = 3,939mm

                // --- CÁLCULO DA INTENSIDADE (mm/h) ---
                // 1 hora = 3.600.000 ms
                // Constante = 0.8005 mm * 3.600.000 ms = 2.881.800
                // Tempo médio de 1 tombada = tempo_2_tombadas_ms / 2.0
                // Como um mm = L²/m...
                if (media_tempo_pluv > 0.0f) {
                    // Constante = 0.8005 mm * 3.600.000 ms/h = 2881800.0f
                    float intensidade_chuva_mm_h = 2881800.0f / media_tempo_pluv;
                    
                    ESP_LOGI(TAG_PLUV, "Tempo 2 ciclos: %.2f ms (%.3f s) | Média/ciclo: %.2f ms", 
                             tempo_2ciclos, 
                             tempo_2ciclos / 1000.0f, 
                             media_tempo_pluv);
                    ESP_LOGI(TAG_PLUV, "Intensidade da precipitação: %.2f mm/h (L/m²/h)", intensidade_chuva_mm_h);
                        intensidade_chuva_mm_h = 0;
                }
                media_tempo_pluv = 0;


                contador_10s = !contador_10s;
}}}}