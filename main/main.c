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
#include "hc05.h"

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

// Structure for controller data
typedef struct {
    int8_t x1_axis;     // Joystick 1 X-axis value (-127 to +127)
    int8_t y1_axis;     // Joystick 1 Y-axis value (-127 to +127)
    int8_t x2_axis;     // Joystick 2 X-axis value (-127 to +127)
    int8_t y2_axis;     // Joystick 2 Y-axis value (-127 to +127)
    uint8_t buttons;    // Bitfield representing button states
} controller_data_t;

// Structure for button events
typedef struct {
    uint gpio;
    bool pressed;
} btn_event_t;

// Global variables
QueueHandle_t xQueueBtnEvents;
SemaphoreHandle_t xSemaphoreControllerData;
controller_data_t controller_data;

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

        // Update controller data
        xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
        controller_data.x1_axis = x_axis;
        xSemaphoreGive(xSemaphoreControllerData);

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

        // Update controller data
        xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
        controller_data.y1_axis = y_axis;
        xSemaphoreGive(xSemaphoreControllerData);

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

        // Update controller data
        xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
        controller_data.x2_axis = x_axis;
        xSemaphoreGive(xSemaphoreControllerData);

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

        // Update controller data
        xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
        controller_data.y2_axis = y_axis;
        xSemaphoreGive(xSemaphoreControllerData);

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
                // Update button bitfield
                xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
                if (event.pressed) {
                    controller_data.buttons |= (1 << btn_index);
                    // If button 1 is pressed, turn on the LED
                    if (btn_index == 0) {
                        gpio_put(LED_PIN, 1); // Turn on LED
                    }
                } else {
                    controller_data.buttons &= ~(1 << btn_index);
                    // If button 1 is released, turn off the LED
                    if (btn_index == 0) {
                        gpio_put(LED_PIN, 0); // Turn off LED
                    }
                }
                xSemaphoreGive(xSemaphoreControllerData);
            }
        }
    }
}

// Task to send data over Bluetooth using uint8_t packet
void hc05_task(void *p) {
    // Initialize UART for HC-05
    uart_init(hc05_UART_ID, hc05_BAUD_RATE);
    gpio_set_function(hc05_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(hc05_RX_PIN, GPIO_FUNC_UART);

    // Initialize HC-05 module
    hc05_init("aps2_2D3Y", "4242");

    controller_data_t data_to_send;
    uint8_t packet[7]; // Packet size is 7 bytes

    while (1) {
        // Atomically copy controller data
        xSemaphoreTake(xSemaphoreControllerData, portMAX_DELAY);
        data_to_send = controller_data;
        xSemaphoreGive(xSemaphoreControllerData);

        // Assemble the packet
        packet[0] = 0xAA; // Header byte
        packet[1] = (uint8_t)data_to_send.x1_axis; // x1_axis
        packet[2] = (uint8_t)data_to_send.y1_axis; // y1_axis
        packet[3] = (uint8_t)data_to_send.x2_axis; // x2_axis
        packet[4] = (uint8_t)data_to_send.y2_axis; // y2_axis
        packet[5] = data_to_send.buttons;          // buttons
        packet[6] = 0xFF;                          // End of Packet

        // Send the packet over UART
        uart_write_blocking(hc05_UART_ID, packet, 7);

        vTaskDelay(pdMS_TO_TICKS(50)); // Adjust send rate as needed
    }
}

int main() {
    stdio_init_all();

    printf("Starting Video Game Controller...\n");

    // Initialize ADC
    adc_init();

    // Create semaphore to protect controller data
    xSemaphoreControllerData = xSemaphoreCreateBinary();

    // Create queue for button events
    xQueueBtnEvents = xQueueCreate(10, sizeof(btn_event_t));

    // Initialize controller data
    controller_data.x1_axis = 0;
    controller_data.y1_axis = 0;
    controller_data.x2_axis = 0;
    controller_data.y2_axis = 0;
    controller_data.buttons = 0;

    // Create tasks
    xTaskCreate(hc05_task, "UART_Task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task1, "X_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(y_task1, "Y_Task1", 1024, NULL, 1, NULL);
    xTaskCreate(x_task2, "X_Task2", 1024, NULL, 1, NULL);
    xTaskCreate(y_task2, "Y_Task2", 1024, NULL, 1, NULL);
    xTaskCreate(btn_task, "Button_Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (true) {
        // Should never reach here
    }
}
