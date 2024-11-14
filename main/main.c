#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "Capstone M3";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
//Cooler
#define Cooler_Temp         CONFIG_COOLER_TEMP              //Analog In
#define Cooler_Voltage      CONFIG_COOLER_VOLTAGE           //Analog In
#define Cooler_Current      CONFIG_COOLER_CURRENT           //Analog In
#define Cooler_PWM          CONFIG_COOLER_PWM               //Analog Out
#define Cooler_ENABLE       CONFIG_COOLER_ENABLE            //Digital Out
//LEDs
#define Power_LED           CONFIG_POWER_LED                //Digital Out
#define Locked_Status_LED   CONFIG_LOCK_STATUS_LED          //Digital Out
#define Cooling_Status_LED  CONFIG_COOLING_STATUS_LED       //Digital Out
#define PCB_Debug_LED       CONFIG_PCB_DEBUG_LED            //Digital Out
//Status
#define Lid_Status          CONFIG_LID_HALL_SENSOR          //Digital In
#define Filled_Status       CONFIG_COOLER_PRESSURE_SENSOR   //Analog In
//Lock
#define Lock_Control        CONFIG_LOCK_CONTROL             //Digital Out
//Fans
#define Fan_Control         CONFIG_FAN_CONTROL              //Digital Out
//Keypad
#define Keypad_C2           CONFIG_KEYPAD_C2                //Digital In
#define Keypad_R1           CONFIG_KEYPAD_R1                //Digital In
#define Keypad_C1           CONFIG_KEYPAD_C1                //Digital In
#define Keypad_R4           CONFIG_KEYPAD_R4                //Digital In
#define Keypad_C3           CONFIG_KEYPAD_C3                //Digital In
#define Keypad_R3           CONFIG_KEYPAD_R3                //Digital In
#define Keypad_R2           CONFIG_KEYPAD_R2                //Digital In
//ADC
#define ADC_SDA             CONFIG_ADC_SDA
#define ADC_SCL             CONFIG_ADC_SCL



static uint8_t s_led_state = 0;



static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(PCB_Debug_LED, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(PCB_Debug_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PCB_Debug_LED, GPIO_MODE_OUTPUT);
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
