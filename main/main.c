/*
 * Video Game Controller with FreeRTOS and HC-05 Bluetooth Module
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <string.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include "hc06.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define WINDOW_SIZE 5  // Tamanho da janela de média móvel

// Definição dos pinos ADC dos joysticks
#define JOYSTICK1_X_PIN 26  // ADC0
#define JOYSTICK1_Y_PIN 27  // ADC1

// Definição dos pinos GPIO dos botões
#define BTN1_PIN 15   // Interruptor que liga o LED
#define BTN2_PIN 14   // Atirar (botão esquerdo do mouse)
#define BTN3_PIN 13   // Mirar (botão direito do mouse)
#define BTN4_PIN 12   // Pular (barra de espaço)
// Removido: #define BTN5_PIN 11  // Granada (tecla 'Q')

// Definição do pino do LED
#define LED_PIN 7

// Estrutura para dados enviados via fila
typedef struct {
    int type;  // 0=x1_axis, 1=y1_axis, 2=buttons
    int8_t val;
} controller_queue_data_t;

// Estrutura para eventos dos botões
typedef struct {
    uint gpio;
    bool pressed;
} btn_event_t;

typedef struct {
    int8_t x1_axis;     // Valor do eixo X do Joystick 1 (-127 a +127)
    int8_t y1_axis;     // Valor do eixo Y do Joystick 1 (-127 a +127)
    uint8_t buttons;    // Índice do botão pressionado
} controller_data_t;

// Variáveis globais
QueueHandle_t xQueueBtnEvents;
QueueHandle_t xQueueAdc;
SemaphoreHandle_t xSemaphoreLED;

// Prototipação das funções
void x_task1(void *p);
void y_task1(void *p);
void btn_task(void *p);
void hc05_task(void *p);
void led_task(void *p);

// Callback de GPIO para os botões
void btn_callback(uint gpio, uint32_t events) {
    btn_event_t event;
    event.gpio = gpio;
    event.pressed = (events & GPIO_IRQ_EDGE_FALL) ? true : false;
    xQueueSendFromISR(xQueueBtnEvents, &event, 0);
}

// Task para ler o eixo X do Joystick 1
void x_task1(void *p) {
    // Inicializa o ADC para o eixo X do Joystick 1
    adc_gpio_init(JOYSTICK1_X_PIN);  // GPIO26

    int window[WINDOW_SIZE] = {0};  // Janela de média móvel
    int sum = 0;
    int index = 0;
    int data_antigo = 5000;
    while (1) {
        adc_select_input(0); // ADC0
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t x_axis = (int8_t)((avg - 2048) * 127 / 2048);

        // Envia os dados via fila
        controller_queue_data_t data;
        data.type = 0;  // x1_axis
        data.val = x_axis;
        if ((data.val < -80 || data.val > 80) &&(data.val != data_antigo)) {
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
            data_antigo = data.val;
            vTaskDelay(20);
        }
    }
}

// Task para ler o eixo Y do Joystick 1
void y_task1(void *p) {
    // Inicializa o ADC para o eixo Y do Joystick 1
    adc_gpio_init(JOYSTICK1_Y_PIN);  // GPIO27

    int window[WINDOW_SIZE] = {0};  // Janela de média móvel
    int sum = 0;
    int index = 0;
    int data_antigo = 5000;
    while (1) {
        adc_select_input(1); // ADC1
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t y_axis = (int8_t)((avg - 2048) * 127 / 2048);

        // Envia os dados via fila
        controller_queue_data_t data;
        data.type = 1;  // y1_axis
        data.val = y_axis;
        if ((data.val < -80 || data.val > 80) &&(data.val != data_antigo)) {
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
            data_antigo = data.val;
            vTaskDelay(20);
        }

    }
}

// Task para tratar eventos dos botões
void btn_task(void *p) {
    // Inicializa os botões com resistores pull-up
    uint button_pins[] = {BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN};
    for (int i = 0; i < 4; i++) {
        gpio_init(button_pins[i]);
        gpio_set_dir(button_pins[i], GPIO_IN);
        gpio_pull_up(button_pins[i]);
        if (i == 0) {
            gpio_set_irq_enabled_with_callback(button_pins[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &btn_callback);
        } else {
            // Configura interrupções de GPIO para os botões
            gpio_set_irq_enabled(button_pins[i],
                                 GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                 true);
        }
    }

    btn_event_t event;
    uint8_t buttons_state = 0;

    while (1) {
        if (xQueueReceive(xQueueBtnEvents, &event, portMAX_DELAY)) {
            uint btn_index = 0xFF;
            if (event.gpio == BTN1_PIN) {
                btn_index = 0;
            } else if (event.gpio == BTN2_PIN) {
                btn_index = 1;
            } else if (event.gpio == BTN3_PIN) {
                btn_index = 2;
            } else if (event.gpio == BTN4_PIN) {
                btn_index = 3;
            }

            if (btn_index != 0xFF) {
                // Atualiza o bitfield dos botões
                if (event.pressed) {
                    buttons_state |= (1 << btn_index);
                } else {
                    buttons_state &= ~(1 << btn_index);
                }

                // Envia o estado dos botões via fila
                controller_queue_data_t data;
                data.type = 2;  // buttons
                data.val = buttons_state;
                xQueueSend(xQueueAdc, &data, portMAX_DELAY);

                // Se for o botão 1 (LED), sinaliza a led_task via semáforo
                if (btn_index == 0 && event.pressed) {
                    xSemaphoreGive(xSemaphoreLED);
                }
            }
        }
    }
}


// Task para controlar o LED
void led_task(void *p) {
    // Inicializa o pino do LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // LED inicialmente apagado

    bool led_on = false;

    while (1) {
        // Aguarda sinalização do semáforo
        if (xSemaphoreTake(xSemaphoreLED, portMAX_DELAY) == pdTRUE) {
            // Alterna o estado do LED
            led_on = !led_on;
            gpio_put(LED_PIN, led_on ? 1 : 0);
        }
    }
}

// Task para enviar dados via Bluetooth usando uint8_t packet
void hc05_task(void *p) {
    // Inicializa UART para o HC-05
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);

    controller_queue_data_t received_data;
    controller_data_t data_to_send = {0};

    while (1) {
        // Recebe dados da fila
        if (xQueueReceive(xQueueAdc, &received_data, portMAX_DELAY)) {
            // Atualiza data_to_send
            if (received_data.type == 0) {
                data_to_send.x1_axis = received_data.val;
            } else if (received_data.type == 1) {
                data_to_send.y1_axis = received_data.val;
            } else if (received_data.type == 2) {
                data_to_send.buttons = (uint8_t)received_data.val;
            }

            // Monta o pacote
            int8_t packet[5];
            packet[0] = 0xAA; // Byte de cabeçalho
            packet[1] = data_to_send.x1_axis;
            packet[2] = data_to_send.y1_axis;
            packet[3] = data_to_send.buttons;
            packet[4] = 0xFF; // Fim do pacote

            // Envia o pacote via UART
            printf("X1: %d, Y1: %d, Buttons: %d\n", packet[1], packet[2], packet[3]);
            uart_write_blocking(HC06_UART_ID, (uint8_t *)packet, 5);
        }
    }
}

int main() {
    stdio_init_all();

    // Inicializa o ADC
    adc_init();

    // Cria o semáforo binário para a led_task
    xSemaphoreLED = xSemaphoreCreateBinary();
    if (xSemaphoreLED == NULL) {
        printf("Falha ao criar o semáforo do LED!\n");
        while (1);
    }

    // Cria a fila para dados do ADC
    xQueueAdc = xQueueCreate(32, sizeof(controller_queue_data_t));
    if (xQueueAdc == NULL) {
        printf("Falha ao criar a fila de dados do ADC!\n");
        while (1);
    }

    // Cria a fila para eventos dos botões
    xQueueBtnEvents = xQueueCreate(10, sizeof(btn_event_t));
    if (xQueueBtnEvents == NULL) {
        printf("Falha ao criar a fila de eventos dos botões!\n");
        while (1);
    }

    // Cria as tasks
    xTaskCreate(hc05_task, "UART_Task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task1, "X_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(y_task1, "Y_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(btn_task, "Button_Task", 1024, NULL, 1, NULL);
    xTaskCreate(led_task, "LED_Task", 1024, NULL, 1, NULL);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    while (true) {
        // Nunca deve chegar aqui
    }
}
