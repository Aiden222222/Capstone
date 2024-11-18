#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "Capstone M3";
static esp_adc_cal_characteristics_t adc1_chars;
static ledc_channel_config_t cooler_pwm_channel;

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

static uint32_t analog_read(adc1_channel_t adc_channel){
    uint32_t mV = esp_adc_cal_raw_to_voltage(adc1_get_raw(adc_channel), &adc1_chars);
    return mV;
}

//Cooler
#define Cooler_Temp_ADC     (adc1_channel_t)ADC1_CHANNEL_0              //Analog In - ADC1->Channel 0 - GPIO1 
int raw_cooler_Temp;
double cooler_Temp;
static void read_cooler_temp(void){
    raw_cooler_Temp = analog_read(Cooler_Temp_ADC);
    int resistance = (10 * raw_cooler_Temp + 16500) / (4.95 - raw_cooler_Temp);
    cooler_Temp = 0.0000001 * (resistance^2) - 0.005*resistance + 67.3;
}

#define Cooler_Voltage_ADC  (adc1_channel_t)ADC1_CHANNEL_1              //Analog In - ADC1->Channel 1 - GPIO2
int raw_cooler_Voltage;
double cooler_Voltage;
static void read_cooler_voltage(void){
    raw_cooler_Voltage = analog_read(Cooler_Voltage_ADC);
    cooler_Voltage = raw_cooler_Voltage / 272.0;    // 272 mV/V
}

#define Cooler_Current_ADC  (adc1_channel_t)ADC1_CHANNEL_2              //Analog In - ADC1->Channel 2 - GPIO3
int raw_cooler_Current;
double cooler_Current;
static void read_cooler_current(void){
    raw_cooler_Current = analog_read(Cooler_Current_ADC);
    cooler_Current = 0.866 * (raw_cooler_Current / 500.0) + 0.0134;    // 500 mV/A
}

#define Cooler_PWM          (gpio_num_t)CONFIG_COOLER_PWM               //Analog Out - 
int duty_cycle = 50;

#define Cooler_ENABLE       (gpio_num_t)CONFIG_COOLER_ENABLE            //Digital Out
#define Fan_Control         (gpio_num_t)CONFIG_FAN_CONTROL              //Digital Out
bool cooler_enabled = false;
static void enable_cooler(void)
{
    gpio_set_level(Cooler_ENABLE, 1);
    gpio_set_level(Fan_Control, 1);
}
static void disable_cooler(void)
{
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

    //Configure PWM - Timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);
    cooler_pwm_channel.channel = LEDC_CHANNEL_0;
    cooler_pwm_channel.duty = 0;
    cooler_pwm_channel.gpio_num = Cooler_PWM;
    cooler_pwm_channel.speed_mode = LEDC_SPEED_MODE_MAX;
    cooler_pwm_channel.hpoint = 0;
    cooler_pwm_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&cooler_pwm_channel);

    ledc_set_duty(cooler_pwm_channel.speed_mode, cooler_pwm_channel.channel, duty_cycle);
    ledc_update_duty(cooler_pwm_channel.speed_mode, cooler_pwm_channel.channel);


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

static void enable_LED(gpio_num_t led_gpio)
{
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        gpio_set_level(led_gpio, 1);
    }
}
static void disable_LED(gpio_num_t led_gpio)
{
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        gpio_set_level(led_gpio, 0);
    }
}

//Status Sensors
#define Lid_Status          (gpio_num_t)CONFIG_LID_HALL_SENSOR          //Digital In
bool lid_closed = false;
static void check_lid_closed(void){
    lid_closed = gpio_get_level(Lid_Status);
}
#define Filled_Status       (gpio_num_t)CONFIG_COOLER_PRESSURE_SENSOR   //Analog In
bool container_filled = false;
static void check_container_filled(void){
    container_filled = gpio_get_level(Filled_Status);
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
static void lock_container(void)
{
    gpio_set_level(Lock_Control, 1);
    container_locked = true;
}
static void unlock_container(void)
{
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

static uint8_t s_led_state = 0;

//Peripheral Configurations
static void configure_peripherals(void)
{
    //ADC Configuration
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

    //Peripherals
    configure_cooler();
    configure_leds();
    configure_sensors();
    configure_lock();
    configure_keypad();
    ESP_LOGI(TAG, "Peripherals configured to capstone configuration.");
}

void app_main(void)
{
    /* Configure all peripherals */
    configure_peripherals();

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        /* Toggle the LED state */
        s_led_state = !s_led_state;


        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
