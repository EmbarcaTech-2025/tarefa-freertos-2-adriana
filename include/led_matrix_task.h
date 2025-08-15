// include/led_matrix_task.h
#ifndef LED_MATRIX_TASK_H
#define LED_MATRIX_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "car_status_data.h" // Para acessar a estrutura car_status_t

// Definições para a matriz de LEDs
#define LED_MATRIX_PIN      7  // Pino GPIO para o DATA IN da matriz de LEDs WS2812B (NeoPixel)
#define NUM_LEDS_MATRIX     25  // Número de LEDs na sua matriz (assumindo uma tira de 25 pixels)
                                // Se for uma matriz 8x8, ajuste para 64 e adapte a lógica de indexação.

// Estrutura para representar uma cor RGB
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

// Protótipo da função da tarefa de controle da matriz de LEDs
void vLedMatrixTask(void *pvParameters);

#endif // LED_MATRIX_TASK_H