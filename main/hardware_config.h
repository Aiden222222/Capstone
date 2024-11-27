/**
 * @file hardware_config.h
 * @brief Hardware Configuration for MedSafe - Capstone M3
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <stdio.h>

/**
 * Cooler
 */
#define Cooler_Temp_ADC     (adc1_channel_t)ADC1_CHANNEL_0              //Analog In - ADC1->Channel 0 - GPIO1 
#define Cooler_Voltage_ADC  (adc1_channel_t)ADC1_CHANNEL_1              //Analog In - ADC1->Channel 1 - GPIO2
#define Cooler_Current_ADC  (adc1_channel_t)ADC1_CHANNEL_2              //Analog In - ADC1->Channel 2 - GPIO3
#define Cooler_PWM          (gpio_num_t)CONFIG_COOLER_PWM               //Analog Out - 
#define Cooler_ENABLE       (gpio_num_t)CONFIG_COOLER_ENABLE            //Digital Out
#define Fan_Control         (gpio_num_t)CONFIG_FAN_CONTROL              //Digital Out
// Cooler PWM
int duty_cycle = 50;
const uint32_t frequency = 200000;

// Function Declarations
void configure_cooler(void);
void read_cooler_temp(void);
void read_cooler_voltage(void);
void read_cooler_current(void);
void enable_cooler(void);
void disable_cooler(void);

/**
 * LEDs
 */
#define Power_LED           (gpio_num_t)CONFIG_POWER_LED                //Digital Out - Green
#define Locked_Status_LED   (gpio_num_t)CONFIG_LOCK_STATUS_LED          //Digital Out - Red
#define Cooling_Status_LED  (gpio_num_t)CONFIG_COOLING_STATUS_LED       //Digital Out - Blue
#define PCB_Debug_LED       (gpio_num_t)CONFIG_PCB_DEBUG_LED            //Digital Out

// Function Declarations
void configure_leds(void);
void enable_LED(gpio_num_t led_gpio);
void disable_LED(gpio_num_t led_gpio);

/**
 * Status Sensors
 */
#define Lid_Status          (gpio_num_t)CONFIG_LID_HALL_SENSOR          //Digital In
#define Filled_Status       (gpio_num_t)CONFIG_COOLER_PRESSURE_SENSOR   //Digital In

// Function Declarations
void configure_sensors(void);
void check_lid_closed(void);
void check_container_filled(void);

/**
 * Lock
 */
#define Lock_Control        (gpio_num_t)CONFIG_LOCK_CONTROL             //Digital Out

// Function Declarations
void configure_lock(void);
void lock_container(void);
void unlock_container(void);

/**
 * Keypad
 */
#define ROWS 4
#define COLS 3

#define Key_C2                  (gpio_num_t)CONFIG_KEYPAD_C2                //Digital In
#define Key_R1                  (gpio_num_t)CONFIG_KEYPAD_R1                //Digital In
#define Key_C1                  (gpio_num_t)CONFIG_KEYPAD_C1                //Digital In
#define Key_R4                  (gpio_num_t)CONFIG_KEYPAD_R4                //Digital In
#define Key_C3                  (gpio_num_t)CONFIG_KEYPAD_C3                //Digital In
#define Key_R3                  (gpio_num_t)CONFIG_KEYPAD_R3                //Digital In
#define Key_R2                  (gpio_num_t)CONFIG_KEYPAD_R2                //Digital In
// GPIO pins for rows and columns
const gpio_num_t ROW_PINS[ROWS] = {Key_R1, Key_R2, Key_R3, Key_R4};
const gpio_num_t COL_PINS[COLS] = {Key_C1, Key_C2, Key_C3};

// Function Declarations
void configure_keypad(void);

#endif