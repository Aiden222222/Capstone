#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "\n";
static esp_adc_cal_characteristics_t adc1_chars;
static ledc_channel_config_t cooler_pwm_channel;

// Cooler PWM
#define COOLER_TIMER              LEDC_TIMER_0
#define COOLER_MODE               LEDC_LOW_SPEED_MODE
#define COOLER_OUTPUT_IO          (4) // Define the output GPIO
#define COOLER_CHANNEL            LEDC_CHANNEL_0
#define COOLER_DUTY_RES           LEDC_TIMER_4_BIT // Set duty resolution to 13 bits
#define COOLER_DUTY               (10) // Set duty to 70%. (2 ** 4) * 70% = 8
#define COOLER_FREQUENCY          (200000) // Frequency in Hertz. Set frequency at 4 kHz

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

static double analog_read(adc1_channel_t adc_channel){
    double volts = esp_adc_cal_raw_to_voltage(adc1_get_raw(adc_channel), &adc1_chars) / 1000.0;
    return volts;
}

//Cooler
#define Cooler_Temp_ADC     (adc1_channel_t)ADC1_CHANNEL_0              //Analog In - ADC1->Channel 0 - GPIO1 
double raw_cooler_Temp;
double cooler_Temp;
int resistance;
static void read_cooler_temp(void){
    raw_cooler_Temp = analog_read(Cooler_Temp_ADC);
    resistance = (10000 * raw_cooler_Temp + 16500) / (4.950 - raw_cooler_Temp);
    cooler_Temp = (0.0000001*resistance*resistance) - (0.005*resistance) + 67.3;
}

#define Cooler_Voltage_ADC  (adc1_channel_t)ADC1_CHANNEL_1              //Analog In - ADC1->Channel 1 - GPIO2
double cooler_Voltage;
static void read_cooler_voltage(void){
    double raw_cooler_Voltage = analog_read(Cooler_Voltage_ADC);
    cooler_Voltage = raw_cooler_Voltage / .272;    // 272 mV/V
}

#define Cooler_Current_ADC  (adc1_channel_t)ADC1_CHANNEL_2              //Analog In - ADC1->Channel 2 - GPIO3
double cooler_Current;
static void read_cooler_current(void){
    double raw_cooler_Current = analog_read(Cooler_Current_ADC);
    cooler_Current = 0.866 * (raw_cooler_Current / .250) + 0.0134;    // 250 mV/A
}

#define Cooler_PWM          (gpio_num_t)CONFIG_COOLER_PWM               //Analog Out - 
int duty_cycle = 50;

#define Cooler_ENABLE       (gpio_num_t)CONFIG_COOLER_ENABLE            //Digital Out
#define Fan_Control         (gpio_num_t)CONFIG_FAN_CONTROL              //Digital Out
bool cooler_enabled = false;
static void enable_cooler(void){
    gpio_set_level(Cooler_ENABLE, 1);
    gpio_set_level(Fan_Control, 1);
}
static void disable_cooler(void){
    gpio_set_level(Cooler_ENABLE, 0);
    gpio_set_level(Fan_Control, 0);
}

static void configure_cooler(void){
    gpio_reset_pin(Cooler_PWM);
    gpio_reset_pin(Cooler_ENABLE);    
    gpio_reset_pin(Fan_Control);

    gpio_set_direction(Cooler_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(Cooler_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Fan_Control, GPIO_MODE_OUTPUT);

    // Prepare and then apply the COOLER PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = COOLER_MODE,
        .duty_resolution  = COOLER_DUTY_RES,
        .timer_num        = COOLER_TIMER,
        .freq_hz          = COOLER_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the COOLER PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = COOLER_MODE,
        .channel        = COOLER_CHANNEL,
        .timer_sel      = COOLER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = COOLER_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    disable_cooler();
}

//LEDs
#define Power_LED           (gpio_num_t)CONFIG_POWER_LED                //Digital Out
#define Locked_Status_LED   (gpio_num_t)CONFIG_LOCK_STATUS_LED          //Digital Out
#define Cooling_Status_LED  (gpio_num_t)CONFIG_COOLING_STATUS_LED       //Digital Out
#define PCB_Debug_LED       (gpio_num_t)CONFIG_PCB_DEBUG_LED            //Digital Out

static void configure_leds(void){
    gpio_reset_pin(Power_LED);
    gpio_reset_pin(Locked_Status_LED);
    gpio_reset_pin(Cooling_Status_LED);
    gpio_reset_pin(PCB_Debug_LED);

    gpio_set_direction(Power_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(Locked_Status_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(Cooling_Status_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_Debug_LED, GPIO_MODE_OUTPUT);
}

static void enable_LED(gpio_num_t led_gpio){
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        gpio_set_level(led_gpio, 1);
    }
}
static void disable_LED(gpio_num_t led_gpio){
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        gpio_set_level(led_gpio, 0);
    }
}

//Status Sensors
#define Lid_Status          (gpio_num_t)CONFIG_LID_HALL_SENSOR          //Digital In
bool lid_closed = false;
static void check_lid_closed(void){
    lid_closed = !(gpio_get_level(Lid_Status));
}
#define Filled_Status       (gpio_num_t)CONFIG_COOLER_PRESSURE_SENSOR   //Digital In
bool container_filled = false;
static void check_container_filled(void){
    container_filled = !(gpio_get_level(Filled_Status));
}

static void configure_sensors(void){
    gpio_reset_pin(Lid_Status);
    gpio_reset_pin(Filled_Status);

    gpio_set_direction(Lid_Status, GPIO_MODE_INPUT);
    gpio_set_direction(Filled_Status, GPIO_MODE_INPUT);
}

//Lock
#define Lock_Control        (gpio_num_t)CONFIG_LOCK_CONTROL             //Digital Out
bool container_locked = false;
static void lock_container(void){
    gpio_set_level(Lock_Control, 1);
    container_locked = true;
}
static void unlock_container(void){
    gpio_set_level(Lock_Control, 0);
    container_locked = false;
}

static void configure_lock(void){
    gpio_reset_pin(Lock_Control);

    gpio_set_direction(Lock_Control, GPIO_MODE_OUTPUT);
}

//Keypad
#define Keypad_C2           (gpio_num_t)CONFIG_KEYPAD_C2                //Digital In
#define Keypad_R1           (gpio_num_t)CONFIG_KEYPAD_R1                //Digital In
#define Keypad_C1           (gpio_num_t)CONFIG_KEYPAD_C1                //Digital In
#define Keypad_R4           (gpio_num_t)CONFIG_KEYPAD_R4                //Digital In
#define Keypad_C3           (gpio_num_t)CONFIG_KEYPAD_C3                //Digital In
#define Keypad_R3           (gpio_num_t)CONFIG_KEYPAD_R3                //Digital In
#define Keypad_R2           (gpio_num_t)CONFIG_KEYPAD_R2                //Digital In

static void configure_keypad(void){
    gpio_reset_pin(Keypad_C2);
    gpio_reset_pin(Keypad_R1);
    gpio_reset_pin(Keypad_C1);
    gpio_reset_pin(Keypad_R4);
    gpio_reset_pin(Keypad_C3);
    gpio_reset_pin(Keypad_R3);
    gpio_reset_pin(Keypad_R2);

    gpio_set_direction(Keypad_C2, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_R1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_C1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_R4, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_C3, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_R3, GPIO_MODE_OUTPUT);
    gpio_set_direction(Keypad_R2, GPIO_MODE_OUTPUT);
}
//ADC
#define ADC_SDA             (gpio_num_t)CONFIG_ADC_SDA
#define ADC_SCL             (gpio_num_t)CONFIG_ADC_SCL

static uint8_t cooler_on = 0;

//Peripheral Configurations
static void configure_peripherals(void){
    //ADC Configuration
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 1100, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

    //Peripherals
    configure_cooler();
    configure_leds();
    configure_sensors();
    configure_lock();
    configure_keypad();
    ESP_LOGI(TAG, "Peripherals configured to capstone configuration.");
}

static void read_display_status(void){
    read_cooler_temp();
    read_cooler_voltage();
    read_cooler_current();
    check_container_filled();
    check_lid_closed();
    ESP_LOGI(TAG, "Raw Temp: %f\nResistance: %d\nTemp: %f C\n", raw_cooler_Temp, resistance, cooler_Temp);
    ESP_LOGI(TAG, "Current: %f A\nVoltage: %f\n", cooler_Current, cooler_Voltage);
    ESP_LOGI(TAG, "Container: %s\n Lid: %s\n", (container_filled == true) ? "Filled" : "Empty", (lid_closed == true) ? "Closed" : "Open");
}

void app_main(void)
{
    /* Configure all peripherals */
    configure_peripherals();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(COOLER_MODE, COOLER_CHANNEL, COOLER_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(COOLER_MODE, COOLER_CHANNEL));

    cooler_on = true;
    ESP_LOGI(TAG, "Turning cooler on!");
    enable_cooler();

    while (1) {
        read_display_status();
        lock_container();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        unlock_container();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
