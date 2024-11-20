/**
 * @file keypad.c
 * @brief 4x3 Keypad library for ESP-IDF framework
 */

#include "keypad.h"

#include <memory.h>
#include <time.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "hardware_config.h"

// Keypad mapping
const char KEYPAD_MAP[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};

/** \brief Keypad configuration pins*/
static gpio_num_t _keypad_pins[7];

/** \brief Last isr time*/
time_t time_old_isr = 0;

/**
 * @brief Handle keypad click
 * @param [in]args row number
 */
void intr_click_handler(void *args);

/**
 * Configure rows for input with interrupts
 */
void setup_rows() {
    for (int i = 0; i < ROWS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << ROW_PINS[i]),
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };
        gpio_config(&io_conf);

        // Attach ISR to row pins
        gpio_isr_handler_add(ROW_PINS[i], intr_click_handler, (void*)i);
    }
}

/**
 * Configure columns as inputs with pull-ups
 */
void setup_cols() {
    for (int i = 0; i < COLS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << COL_PINS[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
    }
}

/**
 * ISR handler for rows
 */
void intr_click_handler(void* args){
    uint32_t now = esp_timer_get_time() / 1000;  // Current time in ms
    if (now - time_old_isr < KEYPAD_DEBOUNCING) {
        return;  // Ignore interrupts within debounce time
    }
    time_old_isr = now;

    int row = (int)args;

    // Scan columns
    for (int col = 0; col < COLS; col++) {
        // Set the current row low
        gpio_set_level(ROW_PINS[row], 0);

        // Check the column state
        if (gpio_get_level(COL_PINS[col]) == 0) {
            char key = KEYPAD_MAP[row][col];
            xQueueSendFromISR(keypad_queue, &key, NULL);  // Send key to queue
        }

        // Restore row state
        gpio_set_level(ROW_PINS[row], 1);
    }
}