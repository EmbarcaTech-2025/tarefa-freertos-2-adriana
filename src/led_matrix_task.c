// src/led_matrix_task.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "led_matrix_task.h"
#include "car_status_data.h"

// Inclui o programa PIO compilado
#include "ws2818b.pio.h" // Este arquivo será gerado automaticamente pelo CMake!

// A fila do status do carro será acessada globalmente via extern
extern QueueHandle_t xCarStatusQueue;

// Array para armazenar as cores dos LEDs
static rgb_color_t led_colors[NUM_LEDS_MATRIX];

// Função para enviar os dados de cor para os LEDs via PIO
static void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8); // Shift 8 bits para a esquerda (GRB de 24 bits)
                                                // O PIO espera 32 bits, mas o WS2812B usa 24.
                                                // Shifting Left garante que os 24 bits estejam no MSB.
}

// Função para atualizar todos os LEDs da matriz com as cores atuais
static void update_led_matrix(void) {
    for (int i = 0; i < NUM_LEDS_MATRIX; i++) {
        // WS2812B espera GRB (Green, Red, Blue) ao invés de RGB
        uint32_t grb = ((uint32_t)led_colors[i].g << 16) |
                       ((uint32_t)led_colors[i].r << 8)  |
                       led_colors[i].b;
        put_pixel(grb);
    }
}

// Função para definir a cor de um LED específico
static void set_led_color(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < NUM_LEDS_MATRIX) {
        led_colors[index].r = r;
        led_colors[index].g = g;
        led_colors[index].b = b;
    }
}

// Função para apagar todos os LEDs
static void clear_led_matrix(void) {
    for (int i = 0; i < NUM_LEDS_MATRIX; i++) {
        set_led_color(i, 0, 0, 0); // Define a cor preta (desligado)
    }
    update_led_matrix(); // Envia os comandos de "desligar" para os LEDs
}

void vLedMatrixTask(void *pvParameters) {
    printf("LedMatrixTask: Iniciada\n");

    // Inicializa o programa PIO para os LEDs WS2812B
    // pio0: Instância do PIO (pode ser pio0 ou pio1)
    // 0: State Machine (SM) a ser usada (0-3)
    // ws2812b_offset: Offset do programa ws2812b dentro da memória de instruções do PIO
    // LED_MATRIX_PIN: Pino GPIO para o qual o PIO enviará os dados
    // 800000: Frequência do clock do PIO (800kHz para WS2812B)
    // false: Não é RGBW (é RGB)
    uint offset = pio_add_program(pio0, &ws2812b_program);
    ws2812b_program_init(pio0, 0, offset, LED_MATRIX_PIN, 800000, false);

    // Teste inicial: acende todos os LEDs em branco por um momento
    printf("LedMatrixTask: Testando LEDs da matriz...\n");
    for(int i = 0; i < NUM_LEDS_MATRIX; i++) {
        set_led_color(i, 20, 20, 20); // Branco suave
    }
    update_led_matrix();
    vTaskDelay(pdMS_TO_TICKS(1000));
    clear_led_matrix(); // Apaga todos os LEDs
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("LedMatrixTask: Teste concluído\n");

    car_status_t current_car_status;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Atualiza a matriz a cada 100ms

    while (true) {
        // Tenta inspecionar o status mais recente do carro (não remove da fila)
        if (xQueuePeek(xCarStatusQueue, &current_car_status, 0) == pdPASS) {
            clear_led_matrix(); // Começa limpando a matriz

            // Lógica para exibir a marcha na matriz de LEDs
            // Usaremos os primeiros LEDs para indicar a marcha (1 a 5)
            if (current_car_status.current_gear > 0 && current_car_status.current_gear <= 5) {
                // Acende LEDs sequencialmente para indicar a marcha
                for (int i = 0; i < current_car_status.current_gear; i++) {
                    set_led_color(i, 0, 0, 255); // Azul para a marcha
                }
            } else if (current_car_status.current_gear == 0) {
                // Neutro: talvez um LED piscando ou uma cor específica
                set_led_color(0, 255, 255, 0); // Amarelo para Neutro
            }

            // Indicador de ABS na matriz (ex: último LED)
            if (current_car_status.abs_active) {
                // Pisca o LED rapidamente (para uma animação mais complexa, use um contador interno)
                static bool abs_blink_state = false;
                abs_blink_state = !abs_blink_state;
                if (abs_blink_state) {
                    set_led_color(NUM_LEDS_MATRIX - 1, 255, 0, 0); // Vermelho para ABS
                } else {
                    set_led_color(NUM_LEDS_MATRIX - 1, 0, 0, 0); // Desliga
                }
            }

            // Indicador de Airbag na matriz (ex: penúltimo LED)
            if (current_car_status.airbag_deployed) {
                // Seta uma cor sólida e mantém após o acionamento
                set_led_color(NUM_LEDS_MATRIX - 2, 255, 100, 0); // Laranja para Airbag
            }

            // Adicione mais lógicas de indicação aqui:
            // - Velocímetro visual (barra de LEDs que se enche com a velocidade)
            // - RPM (cores diferentes em certos thresholds)
            // - Setas de direção (se você adicionar botões para isso)

            update_led_matrix(); // Envia as cores atualizadas para a matriz de LEDs
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Espera até o próximo ciclo
    }
    // Nunca deve chegar aqui, pois a tarefa é infinita