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

#define WINDOW_SIZE 5  // Size of the moving average window

// Define joystick ADC input pins
#define JOYSTICK1_X_PIN 26  // ADC0
#define JOYSTICK1_Y_PIN 27  // ADC1
#define JOYSTICK2_X_PIN 21  // ADC2
#define JOYSTICK2_Y_PIN 20  // ADC3

// Define button GPIO pins
#define BTN1_PIN 15   // Switch that turns on LED
#define BTN2_PIN 14   // Shooting (left mouse button)
#define BTN3_PIN 13   // Aiming (right mouse button)
#define BTN4_PIN 12  // Jumping (spacebar)
#define BTN5_PIN 11  // Grenade ('Q' key)

// Define LED pin (control ligado)
#define LED_PIN 7

// Structure for data sent via the queue
typedef struct {
    int type;  // 0=x1_axis, 1=y1_axis, 2=x2_axis, 3=y2_axis, 4=buttons
    int8_t val;
} controller_queue_data_t;

// Structure for button events
typedef struct {
    uint gpio;
    bool pressed;
} btn_event_t;

// Global variables
QueueHandle_t xQueueBtnEvents;
QueueHandle_t xQueueAdc;
SemaphoreHandle_t xSemaphoreLED;

// Function prototypes
void x_task1(void *p);
void y_task1(void *p);
void x_task2(void *p);
void y_task2(void *p);
void btn_task(void *p);
void hc05_task(void *p);

// GPIO callback for buttons
void btn_callback(uint gpio, uint32_t events) {
    btn_event_t event;
    event.gpio = gpio;
    event.pressed = (events & GPIO_IRQ_EDGE_FALL) ? true : false;
    xQueueSendFromISR(xQueueBtnEvents, &event, 0);
}

// Task to read X-axis of Joystick 1
void x_task1(void *p) {
    // Initialize ADC for Joystick 1 X-axis
    adc_gpio_init(JOYSTICK1_X_PIN);  // GPIO26

    int window[WINDOW_SIZE] = {0};  // Moving average window
    int sum = 0;
    int index = 0;

    while (1) {
        adc_select_input(0); // ADC0
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t x_axis = (int8_t)((avg - 2048) * 127 / 2048);

       // Send data via queue
        controller_queue_data_t data;
        data.type = 0;  // x1_axis
        data.val = x_axis;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to read Y-axis of Joystick 1
void y_task1(void *p) {
    // Initialize ADC for Joystick 1 Y-axis
    adc_gpio_init(JOYSTICK1_Y_PIN);  // GPIO27

    int window[WINDOW_SIZE] = {0};  // Moving average window
    int sum = 0;
    int index = 0;

    while (1) {
        adc_select_input(1); // ADC1
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t y_axis = (int8_t)((avg - 2048) * 127 / 2048);

        // Send data via queue
        controller_queue_data_t data;
        data.type = 1;  // y1_axis
        data.val = y_axis;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to read X-axis of Joystick 2
void x_task2(void *p) {
    // Initialize ADC for Joystick 2 X-axis
    adc_gpio_init(JOYSTICK2_X_PIN);  // GPIO28

    int window[WINDOW_SIZE] = {0};  // Moving average window
    int sum = 0;
    int index = 0;

    while (1) {
        adc_select_input(2); // ADC2
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t x_axis = (int8_t)((avg - 2048) * 127 / 2048);
        
        // Send data via queue
        controller_queue_data_t data;
        data.type = 2;  // x2_axis
        data.val = x_axis;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to read Y-axis of Joystick 2
void y_task2(void *p) {
    // Initialize ADC for Joystick 2 Y-axis
    adc_gpio_init(JOYSTICK2_Y_PIN);  // GPIO29

    int window[WINDOW_SIZE] = {0};  // Moving average window
    int sum = 0;
    int index = 0;

    while (1) {
        adc_select_input(3); // ADC3
        int val = adc_read();
        sum -= window[index];
        window[index] = val;
        sum += window[index];
        index = (index + 1) % WINDOW_SIZE;
        int avg = sum / WINDOW_SIZE;

        int8_t y_axis = (int8_t)((avg - 2048) * 127 / 2048);

        // Send data via queue
        controller_queue_data_t data;
        data.type = 3;  // y2_axis
        data.val = y_axis;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to handle button events
void btn_task(void *p) {
    // Initialize buttons with pull-up resistors
    uint button_pins[] = {BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN, BTN5_PIN};
    for (int i = 0; i < 5; i++) {
        gpio_init(button_pins[i]);
        gpio_set_dir(button_pins[i], GPIO_IN);
        gpio_pull_up(button_pins[i]);

        // Configure GPIO interrupts for buttons
        gpio_set_irq_enabled_with_callback(button_pins[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &btn_callback);
    }

    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // LED initially off 

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
            } else if (event.gpio == BTN5_PIN) {
                btn_index = 4;
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
                data.type = 4;  // buttons
                data.val = buttons_state;
                xQueueSend(xQueueAdc, &data, portMAX_DELAY);

                // Se for o botão 1, sinaliza a led_task via semáforo
                if (btn_index == 0) {
                    xSemaphoreGive(xSemaphoreLED);
                }
            }
        }
    }
}

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

// Task to send data over Bluetooth using uint8_t packet
void hc05_task(void *p) {
    // Initialize UART for HC-05
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);

    // Initialize HC-05 module
    //hc06_init("aps2-af", "4242");

    controller_queue_data_t received_data;

    //uart_putc_raw(HC06_UART_ID, 'a');

     while (1) {
        // Recebe dados da fila
        if (xQueueReceive(xQueueAdc, &received_data, portMAX_DELAY)) {
            uint8_t packet[7];
            packet[0] = 0xAA; // Byte de cabeçalho
            if (received_data.type == 0) {
                packet[1] = (uint8_t)received_data.val;
            } else if (received_data.type == 1) {
                packet[2] = (uint8_t)received_data.val;
            } else if (received_data.type == 2) {
                packet[3] = (uint8_t)received_data.val;
            } else if (received_data.type == 3) {
                packet[4] = (uint8_t)received_data.val;
            } else if (received_data.type == 4) {
                packet[5] = received_data.val;
            }
            packet[6] = 0xFF; // Fim do pacote

            // Envia o pacote via UART
            uart_write_blocking(HC06_UART_ID, packet, 7);
        }
    }
}


int main() {
    stdio_init_all();

    printf("Starting Video Game Controller...\n");

    // Initialize ADC
    adc_init();

    // Create semaphore to protect controller data
    xSemaphoreLED = xSemaphoreCreateBinary();
    xQueueAdc = xQueueCreate(32, sizeof(controller_queue_data_t));

    // Create queue for button events
    xQueueBtnEvents = xQueueCreate(10, sizeof(btn_event_t));

    // Create tasks
    xTaskCreate(hc05_task, "UART_Task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task1, "X_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(y_task1, "Y_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(x_task2, "X_Task2", 1024, NULL, 1, NULL);
    xTaskCreate(y_task2, "Y_Task2", 1024, NULL, 1, NULL);
    xTaskCreate(btn_task, "Button_Task", 1024, NULL, 1, NULL);
    xTaskCreate(led_task, "LED_Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (true) {
        // Should never reach here
    }
}
