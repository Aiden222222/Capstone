#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "sdkconfig.h"
#include "keypad.h"
#include "hardware_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "inttypes.h"
#include "env_config.h"

#define SENSOR_TASK_INTERVAL_MS 5000


static const char *TAG = "MedSafe\n";
static esp_adc_cal_characteristics_t adc1_chars;

static double analog_read(adc1_channel_t adc_channel){
    double volts = esp_adc_cal_raw_to_voltage(adc1_get_raw(adc_channel), &adc1_chars) / 1000.0;
    return volts;
}

//Cooler
static void configure_cooler(void){
    gpio_reset_pin(Cooler_PWM);
    gpio_reset_pin(Cooler_ENABLE);    
    gpio_reset_pin(Fan_Control);

    gpio_set_direction(Cooler_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(Cooler_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Fan_Control, GPIO_MODE_OUTPUT);

    // Prepare and then apply the COOLER PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_4_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = frequency,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the COOLER PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = Cooler_PWM,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    disable_cooler();
}

double cooler_Temp;
static void read_cooler_temp(void){
    double raw_cooler_Temp = analog_read(Cooler_Temp_ADC);
    int resistance = (10000 * raw_cooler_Temp + 16500) / (4.950 - raw_cooler_Temp);
    cooler_Temp = (0.0000001*resistance*resistance) - (0.005*resistance) + 67.3;
}
double cooler_Voltage;
static void read_cooler_voltage(void){
    double raw_cooler_Voltage = analog_read(Cooler_Voltage_ADC);
    cooler_Voltage = raw_cooler_Voltage / .272;    // 272 mV/V
}
double cooler_Current;
static void read_cooler_current(void){
    double raw_cooler_Current = analog_read(Cooler_Current_ADC);
    cooler_Current = 0.866 * (raw_cooler_Current / .250) + 0.0134;    // 250 mV/A
}
bool cooler_enabled = false;
static void enable_cooler(void){
    gpio_set_level(Cooler_ENABLE, 1);
    gpio_set_level(Fan_Control, 1);
    cooler_enabled = true;
}
static void disable_cooler(void){
    gpio_set_level(Cooler_ENABLE, 0);
    gpio_set_level(Fan_Control, 0);
    cooler_enabled = false;

}

//LED
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
static void configure_sensors(void){
    gpio_reset_pin(Lid_Status);
    gpio_reset_pin(Filled_Status);

    gpio_set_direction(Lid_Status, GPIO_MODE_INPUT);
    gpio_set_direction(Filled_Status, GPIO_MODE_INPUT);
}

bool lid_closed = false;
static void check_lid_closed(void){
    lid_closed = !(gpio_get_level(Lid_Status));
}
bool container_filled = false;
static void check_container_filled(void){
    container_filled = !(gpio_get_level(Filled_Status));
}

//Lock
static void configure_lock(void){
    gpio_reset_pin(Lock_Control);
    gpio_set_direction(Lock_Control, GPIO_MODE_OUTPUT);
}

bool container_locked = false;
static void lock_container(void){
    gpio_set_level(Lock_Control, 0);
    container_locked = true;
}
static void unlock_container(void){
    gpio_set_level(Lock_Control, 1);
    container_locked = false;
}

//Keypad
/** \brief Pressed keys queue*/
QueueHandle_t keypad_queue;

static void configure_keypad(void){
    gpio_reset_pin(Key_C2);
    gpio_reset_pin(Key_R1);
    gpio_reset_pin(Key_C1);
    gpio_reset_pin(Key_R4);
    gpio_reset_pin(Key_C3);
    gpio_reset_pin(Key_R3);
    gpio_reset_pin(Key_R2);

    setup_rows();
    setup_cols();
}


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


// TASKS
/**
 * Task to handle key presses
 */
void keypad_task(void* arg) {
    char key;
    while (1) {
        // Wait for a key press from the queue
        if (xQueueReceive(keypad_queue, &key, portMAX_DELAY)) {
            printf("Key Pressed: %c\n", key);
        }
    }
}

static void read_display_status_task(void){
    while (1) {
        read_cooler_temp();
        read_cooler_voltage();
        read_cooler_current();
        check_container_filled();
        check_lid_closed();
        
        // Log the measurement
        ESP_LOGI(TAG, "Temp: %f C\nCurrent: %f A\nVoltage: %f\nContainer: %s\n Lid: %s\n", cooler_Temp, cooler_Current, cooler_Voltage, (container_filled == true) ? "Filled" : "Empty", (lid_closed == true) ? "Closed" : "Open");

        // Delay for the defined interval
        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_INTERVAL_MS));
    }
}

void app_main(void)
{
    /* Configure all peripherals */
    configure_peripherals();
    // Set duty to 50%
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (floor((pow(2,4))*(duty_cycle / 100.0))));
    // Update duty to apply the new value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    gpio_install_isr_service(0);
    xTaskCreate(keypad_task, "keypad_task", 2048, NULL, 5, NULL);
    printf("Keypad initialized. Waiting for key presses...\n");
    xTaskCreate(read_display_status_task, "update_status_task", 2048, NULL, 4, NULL);

    cooler_enabled = true;
    ESP_LOGI(TAG, "Turning cooler on!");
    enable_cooler();

    while (1) {
        read_display_status_task();
        unlock_container();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}







// ESP Wifi and server connection
// static const char *TAG = "ESP32_MQTT";

uint32_t MQTT_CONNECTED = 0;
static esp_mqtt_client_handle_t client = NULL;
static float setpoint = 10.0; // Default setpoint in Celsius

static void mqtt_app_start(void);
static void wifi_init(void);

/**
 * @brief Handles Wi-Fi and IP events.
 * Connects to Wi-Fi when STA starts, reconnects if disconnected, and
 * starts the MQTT client once an IP is obtained.
 * 
 * @param arg        Pointer to arguments for the event handler.
 * @param event_base Event base identifier.
 * @param event_id   Event ID.
 * @param event_data Event data.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP, starting MQTT client");
        mqtt_app_start();
    }
}

/**
 * @brief Handles various MQTT events.
 * Responds to events like connection, disconnection, data reception, and errors.
 * Logs messages for each event, subscribes to a topic when connected, and displays
 * topic and data information upon receiving a message.
 *
 * @param handler_args Arguments passed to the handler.
 * @param base         Event base identifier.
 * @param event_id     Event ID.
 * @param event_data   Event data.
 */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    int msg_id;

    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            MQTT_CONNECTED = 1;
            msg_id = esp_mqtt_client_subscribe(client, "/Medsafe/setpoint", 0);
            ESP_LOGI(TAG, "Subscribed to /Medsafe/setpoint, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            MQTT_CONNECTED = 0;
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            if (strncmp(event->topic, "/Medsafe/setpoint", event->topic_len) == 0) {
                char setpoint_str[16] = {0};
                strncpy(setpoint_str, event->data, event->data_len);
                setpoint_str[event->data_len] = '\0'; // Null- terminate the string
                setpoint = atof(setpoint_str);  // Update the setpoint
                ESP_LOGI(TAG, "Updated setpoint: %.2f°C", setpoint);
            } else {
                ESP_LOGI(TAG, "Received message on an unexpected topic: %.*s", event->topic_len, event->topic);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Unhandled event id:%" PRId32, event_id);
            break;
    }
}

/**
 * @brief Configures and starts the MQTT client.
 * Initializes the MQTT client with server configuration and registers the
 * event handlers for MQTT events such as connection, data reception, and errors.
 */

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_config = {
    .broker.address.uri = "mqtt://test.mosquitto.org:1883"
    };
    
    client = esp_mqtt_client_init(&mqtt_config);
// Register the event handler for specific MQTT events
    esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DATA, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ERROR, mqtt_event_handler, client);    esp_mqtt_client_start(client);
}

// Adjust the cooler temperature
void adjust_cooler(float current_temp, float setpoint) {
    if (current_temp > setpoint) {
        // Activate cooling
        enable_cooler();
        ESP_LOGI(TAG, "Cooling activated. Target: %.2f°C, Current: %.2f°C", setpoint, current_temp);
    } else {
        // Deactivate cooling
        disable_cooler();
        ESP_LOGI(TAG, "Cooling deactivated. Target: %.2f°C, Current: %.2f°C", setpoint, current_temp);
    }
}


/**
 * Initializes the Wi-Fi connection for the ESP32 in station mode.
 * Configures Wi-Fi settings, registers event handlesrs, and starts the Wi-FI
 */
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {};
    strcpy((char *) wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *) wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/**
 * @brief Publishes sensor data periodically when MQTT is connected.
 * Checks if MQTT is connected before publishing. Sends sensor data to a specified topic
 * every 15 seconds. Logs the result of each publish attempt.
 * 
 * @param params Parameters for the task.
 */

void Publisher_Task(void *params) {
    while (true) {
        if (MQTT_CONNECTED) {

            float current_temp = read_cooler_temp(); // Update cooler_Temp
            adjust_cooler(current_temp, setpoint); //Adjust cooler based on setpoint
            
            check_container_filled(); // Update container_filled

            // Prepare payload
            char payload[64];
            snprintf(payload, sizeof(payload),
                    "{f\"temperature\": %.2f, \"container_filled\": %s}",
                    current_temp, container_filled ? "true" : "false");
            
            // Publish data
            float msg_id = esp_mqtt_client_publish(client, "/Medsafe/data", payload, 0, 0, 0);
            ESP_LOGI(TAG, "Published data: %s", payload);
            
            } else {
                ESP_LOGW(TAG, "MQTT not connected, unable to publish");
            }
            vTaskDelay(15000 / portTICK_PERIOD_MS); // Publish every 15 seconds
        } 
        
    }

/**
 * @brief Main entry point of the application.
 * Initializes NVS, Wi-Fi, and starts the publisher task to handle MQTT data publishing.
 */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
}