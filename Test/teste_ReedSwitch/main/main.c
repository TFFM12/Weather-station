#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "REED_SWITCH";

// Configuração do pino do Reed Switch
#define REED_SWITCH_GPIO        GPIO_NUM_4  
#define DEBOUNCE_TIME_MS        50          // Tempo para ignorar o ruído do contacto mecânico

// Fila para passar eventos de interrupção da ISR para a tarefa
static QueueHandle_t gpio_evt_queue = NULL;

/**
 * @brief Interrupção (ISR) executada de cada vez que o pino deteta uma alteração de estado
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // Envia o número do pino para a fila sem bloquear a interrupção
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief Tarefa dedicada a processar as rotações e aplicar o debounce
 */
static void reed_switch_task(void* arg)
{
    uint32_t io_num;
    uint32_t rotacoes = 0;
    int64_t ultimo_tempo_ms = 0;

    while (1) {
        // Bloqueia e aguarda até que a ISR envie um evento para a fila
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t tempo_atual_ms = esp_timer_get_time() / 1000;

            // Filtro de Debounce: valida a ativação apenas se tiver passado o tempo mínimo
            if ((tempo_atual_ms - ultimo_tempo_ms) > DEBOUNCE_TIME_MS) {
                // Como o pino está em PULL-UP, o contacto fecha ligando a GND (nível LOW)
                if (gpio_get_level(io_num) == 0) {
                    rotacoes++;
                    ESP_LOGI(TAG, "Nº Rotações %lu", rotacoes);
                    ultimo_tempo_ms = tempo_atual_ms;
                }
            }
        }
    }
}

/**
 * @brief Configura o GPIO do Reed Switch
 */
static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << REED_SWITCH_GPIO), // Seleciona o pino GPIO
        .mode = GPIO_MODE_INPUT,                    // Configura como Entrada
        .pull_up_en = GPIO_PULLUP_ENABLE,           // Ativa o Pull-Up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Desativa o Pull-Down
        .intr_type = GPIO_INTR_NEGEDGE,             // Interrupção na transição HIGH -> LOW (quando o reed fecha)
    };
    gpio_config(&io_conf);

    // Cria a fila do FreeRTOS com capacidade para 10 eventos
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Instala o serviço genérico de interrupções de GPIO do ESP-IDF
    gpio_install_isr_service(0);
    // Associa a ISR ao pino configurado
    gpio_isr_handler_add(REED_SWITCH_GPIO, gpio_isr_handler, (void*) REED_SWITCH_GPIO);
}

void app_main(void)
{
    ESP_LOGI(TAG, "A inicializar o leitor de Reed Switch...");

    // Inicializa o hardware GPIO e interrupções
    gpio_init();

    // Cria uma tarefa FreeRTOS dedicada com prioridade 10
    xTaskCreate(reed_switch_task, "reed_switch_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Pronto! A aguardar rotações...");
}