/**
 * @file keypad.h
 * @brief 4x3 Keypad library for ESP-IDF framework
 */

#ifndef KEYPAD_H
#define KEYPAD_H

#include <driver/gpio.h>

#define KEYPAD_DEBOUNCING 100   ///< time in ms
#define KEYPAD_STACKSIZE  5

/**
 * @brief Initialize Keypad settings and start it, setup up directions and isr and initialize
 * key pressed queue.
 * 
 * @param keypad_pins Keypad Connections Array following this template: 
 *  {R1, R2, R3, R4, C1, C2, C3}
 * 
 * @return esp_err_t returns ESP_OK if succeful initialize
 */
esp_err_t keypad_initalize(gpio_num_t keypad_pins[7]);


#endif