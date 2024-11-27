#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <time.h>
#include <ctype.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "inttypes.h"
#include "hardware_config.h"
#include "env_config.h"
#include "keypad.h"


static const char *TAG = "MedSafe";
static const char *TAG_HW = "HW";
static const char *TAG_WIFI = "WiFi";
static const char *TAG_HTTP = "HTTP";
static const char *TAG_MQTT = "MQTT";

int8_t setup_complete = 0;

static nvs_handle_t my_handle;
static TimerHandle_t entry_timer;
esp_err_t err;
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/**
 *HARDWARE 
 */
static esp_adc_cal_characteristics_t adc1_chars;
static double analog_read(adc1_channel_t adc_channel){
    double volts = esp_adc_cal_raw_to_voltage(adc1_get_raw(adc_channel), &adc1_chars) / 1000.0;
    return volts;
}

//Cooler
void configure_cooler(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Cooler_PWM));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Cooler_ENABLE));    
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Fan_Control));

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Cooler_PWM, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Cooler_ENABLE, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Fan_Control, GPIO_MODE_OUTPUT));

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
void read_cooler_temp(void){
    double raw_cooler_Temp = analog_read(Cooler_Temp_ADC);
    int resistance = (10000 * raw_cooler_Temp + 16500) / (4.950 - raw_cooler_Temp);
    cooler_Temp = (0.0000001*resistance*resistance) - (0.005*resistance) + 67.3;
}
double cooler_Voltage;
void read_cooler_voltage(void){
    double raw_cooler_Voltage = analog_read(Cooler_Voltage_ADC);
    cooler_Voltage = raw_cooler_Voltage / .272;    // 272 mV/V
}
double cooler_Current;
void read_cooler_current(void){
    double raw_cooler_Current = analog_read(Cooler_Current_ADC);
    cooler_Current = 0.866 * (raw_cooler_Current / .250) + 0.0134;    // 250 mV/A
}
bool cooler_enabled = false;
static void update_duty_cycle(int duty){
    if(duty > 70){
        ESP_LOGW(TAG_HW, "Max Duty Cycle is 70 percent!\n");
        duty_cycle = 70;
    } else if(duty < 0){
        duty_cycle = 0;
    } else {
        duty_cycle = duty;
    }

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (floor((pow(2,4))*(duty_cycle / 100.0)))));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
void enable_cooler(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Cooler_ENABLE, 1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Fan_Control, 1));
    enable_LED(Cooling_Status_LED);
    cooler_enabled = true;
}
void disable_cooler(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Cooler_ENABLE, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Fan_Control, 0));
    disable_LED(Cooling_Status_LED);
    update_duty_cycle(0);
    cooler_enabled = false;
}


//LED
void configure_leds(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Power_LED));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Locked_Status_LED));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Cooling_Status_LED));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(PCB_Debug_LED));

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Power_LED, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Locked_Status_LED, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Cooling_Status_LED, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(PCB_Debug_LED, GPIO_MODE_OUTPUT));
}
void enable_LED(gpio_num_t led_gpio){
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(led_gpio, 1));
    }
}
void disable_LED(gpio_num_t led_gpio){
    if(GPIO_IS_VALID_GPIO(led_gpio)){
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(led_gpio, 0));
    }
}

//Status Sensors
#define SENSOR_TASK_INTERVAL_MS 30000
void configure_sensors(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Lid_Status));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Filled_Status));

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Lid_Status, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Filled_Status, GPIO_MODE_INPUT));
}
bool lid_closed = false;
void check_lid_closed(void){
    lid_closed = !(gpio_get_level(Lid_Status));
}
bool container_filled = false;
void check_container_filled(void){
    container_filled = !(gpio_get_level(Filled_Status));
}

//Lock
void configure_lock(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Lock_Control));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(Lock_Control, GPIO_MODE_OUTPUT));
}
bool container_locked = true;
void lock_container(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Lock_Control, 0));
    enable_LED(Locked_Status_LED);
    container_locked = true;
}
void unlock_container(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(Lock_Control, 1));
    disable_LED(Locked_Status_LED);
    container_locked = false;
}

//Keypad
QueueHandle_t keypad_queue;
time_t last_key_time = 0;
char pin_buffer[5] = {0};
int pin_index = 0;
void setup_rows(void) {
    for (int i = 0; i < ROWS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << ROW_PINS[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        ESP_ERROR_CHECK(gpio_set_level(ROW_PINS[i], 1));
    }
}
void setup_cols(void) {
    for (int i = 0; i < COLS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << COL_PINS[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}
void configure_keypad(void){
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_C2));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_R1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_C1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_R4));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_C3));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_R3));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(Key_R2));

    keypad_queue = xQueueCreate(5, sizeof(char));

    setup_rows();
    setup_cols();

    ESP_LOGD(TAG_HW, "Keypad initialized. Waiting for key presses...\n");
}

//Peripheral Configurations
static void configure_peripherals(void){

    //ADC Configuration
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_DEFAULT, 1100, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));

    //Peripherals
    configure_cooler();
    configure_leds();
    configure_sensors();
    configure_lock();
    configure_keypad();
    ESP_LOGI(TAG_HW, "Peripherals configured to capstone configuration.");

    enable_LED(Power_LED);
}

/**
 * WIFI AND SERVER CONNECTIONS
 */
bool MQTT_CONNECTED = false;
bool WIFI_CONNECTED = false;
bool data_received = false;
static esp_mqtt_client_handle_t client = NULL;
static esp_netif_t *ap_netif;  // AP interface
static esp_netif_t *sta_netif; // STA interface

static double desired_temp = 10.0;

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
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    int msg_id;

    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGD(TAG_MQTT, "MQTT_EVENT_CONNECTED");
            MQTT_CONNECTED = 1;
            msg_id = esp_mqtt_client_subscribe(client, "/Medsafe/setpoint", 0);
            ESP_LOGD(TAG_MQTT, "Subscribed to /Medsafe/setpoint, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
            MQTT_CONNECTED = 0;
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG_MQTT, "MQTT_EVENT_DATA");
            if (strncmp(event->topic, "/Medsafe/setpoint", event->topic_len) == 0) {
                char temp_str[64] = {0};
                strncpy(temp_str, event->data, event->data_len);
                temp_str[event->data_len] = '\0'; // Null- terminate the string
                desired_temp = atof(temp_str);  // Update the setpoint
                ESP_LOGD(TAG_MQTT, "Updated temperature: %.2f°C", desired_temp);
            } else {
                ESP_LOGW(TAG_MQTT, "Received message on an unexpected topic: %.*s", event->topic_len, event->topic);
            }
            data_received = true;
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG_MQTT, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGW(TAG_MQTT, "Unhandled event id:%" PRId32, event_id);
            break;
    }
}

/**
 * @brief Configures and starts the MQTT client.
 * Initializes the MQTT client with server configuration and registers the
 * event handlers for MQTT events such as connection, data reception, and errors.
 */
static void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_config = {
    .broker.address.uri = "mqtt://test.mosquitto.org:1883"
    };
    
    client = esp_mqtt_client_init(&mqtt_config);
    // Register the event handler for specific MQTT events
    esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DATA, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ERROR, mqtt_event_handler, client);
    
    esp_mqtt_client_start(client);

    ESP_LOGD(TAG_MQTT, "MQTT client started!");
}

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
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGD(TAG_WIFI, "WiFi started, connecting to WiFi.");
        err = esp_wifi_connect();
        WIFI_CONNECTED = (err == ESP_OK);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG_WIFI, "Wi-Fi disconnected, reconnecting...");
        WIFI_CONNECTED = false;
        err = esp_wifi_connect();
        WIFI_CONNECTED = (err == ESP_OK);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGD(TAG_WIFI, "Got IP, starting MQTT client");
        mqtt_app_start();
    }
}

static void wifi_init_ap(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_config = {};
    strcpy((char *) ap_config.ap.ssid, CONFIG_ESP_AP_SSID);
    strcpy((char *) ap_config.ap.password, ESP_AP_PASSWORD);
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.ssid_hidden = 0;

    if (strlen((char *) ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = ESP_IP4TOADDR(192, 168, 4, 2);
    ip_info.gw.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    esp_netif_dhcps_start(ap_netif);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGD(TAG_WIFI, "WiFi AP Init finished. SSID:%s password:%s", CONFIG_ESP_AP_SSID, ESP_AP_PASSWORD); 
}

/**
 * Initializes the Wi-Fi connection for the ESP32 in station mode.
 * Configures Wi-Fi settings, registers event handlesrs, and starts the Wi-FI
 */
static void wifi_init_sta(void){
    // Create STA network interface
    if (!sta_netif) {
        sta_netif = esp_netif_create_default_wifi_sta();
    }
    if(WIFI_CONNECTED){
        esp_wifi_disconnect();
        WIFI_CONNECTED = false;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t sta_config = {};
    strcpy((char *) sta_config.sta.ssid, WIFI_SSID);
    strcpy((char *) sta_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGD(TAG_WIFI, "WiFi STA Init finished.");
}

// HTTP handler for serving the form
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Helper function to URL-decode a string
static void url_decode(char *dest, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b))) {
            a = (a >= 'a') ? a - 'a' + 10 : (a >= 'A') ? a - 'A' + 10 : a - '0';
            b = (b >= 'a') ? b - 'a' + 10 : (b >= 'A') ? b - 'A' + 10 : b - '0';
            *dest++ = 16 * a + b;
            src += 3;
        } else if (*src == '+') {
            *dest++ = ' '; // Replace '+' with space
            src++;
        } else {
            *dest++ = *src++;
        }
    }
    *dest = '\0';
}

// HTTP handler for processing the form data
static esp_err_t setconfig_post_handler(httpd_req_t *req) {
    char buf[1024];
    int ret, remaining = req->content_len;

    // Read the form data
    while (remaining > 0) {
        ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return ESP_FAIL;
        }
        remaining -= ret;
        buf[ret] = '\0'; // Null-terminate the buffer
        ESP_LOGD(TAG_HTTP, "Received form data: %s", buf);
    }

    // Parse key-value pairs from the URL-encoded form data
    char *key, *value;
    char *pair = strtok(buf, "&");
    while (pair) {
        key = strtok(pair, "=");
        value = strtok(NULL, "=");
        if (key && value) {
            // Decode and assign values to respective variables
            if (strcmp(key, "wifi_ssid") == 0) {
                url_decode(&WIFI_SSID, value);
            } else if (strcmp(key, "wifi_pw") == 0) {
                url_decode(&WIFI_PASSWORD, value);
            } else if (strcmp(key, "reset") == 0) {
                url_decode(&reset_pin, value);
                reset_pin[4] = "\0";
            } else if (strcmp(key, "lock") == 0) {
                url_decode(&lock_pin, value);
                lock_pin[4] = "\0";
            } else if (strcmp(key, "ap_pw") == 0) {
                url_decode(&ESP_AP_PASSWORD, value);
            }
        }
        pair = strtok(NULL, "&");
    }

    ESP_LOGD(TAG_HTTP, "Received Config: Wifi SSID = %s, Wifi PW = %s, Reset Pin = %s, Lock Pin = %s, AP PW = %s", WIFI_SSID, WIFI_PASSWORD, reset_pin, lock_pin, ESP_AP_PASSWORD);

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(my_handle, "WiFi_SSID", WIFI_SSID));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(my_handle, "WiFi_PASSWORD", WIFI_PASSWORD));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(my_handle, "Reset_Pin", reset_pin));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(my_handle, "Lock_Pin", lock_pin));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(my_handle, "ESP_AP_PASSWORD", ESP_AP_PASSWORD));
    setup_complete = 1;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_i8(my_handle, "setup_complete", setup_complete));

    // Respond to the client
    httpd_resp_sendstr(req, "<h1>Configuration Saved Successfully!</h1>");
    wifi_init_sta();
    return ESP_OK;
}

// Start the HTTP server
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t setconfig = {
            .uri = "/setconfig",
            .method = HTTP_POST,
            .handler = setconfig_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &setconfig);

        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return NULL;
}

static void setup_MedSafe(void){
    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_init_ap();
    start_webserver();

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_get_i8(my_handle, "setup_complete", &setup_complete);
        ESP_LOGD(TAG, "SETUP_COMPLETE CHECK: %d", setup_complete);
        if(err == ESP_OK && setup_complete == 1){ //Setup complete --> Read values from NVS
            size_t SSID_LENGTH_t =  MAX_SSID_LENGTH;
            size_t PW_LENGTH_t =  MAX_PW_LENGTH;
            size_t keypad_pin_len_t =  keypad_pin_len;
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, "WiFi_SSID", &WIFI_SSID, &SSID_LENGTH_t));
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, "WiFi_PASSWORD", &WIFI_PASSWORD, &PW_LENGTH_t));
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, "Lock_Pin", &lock_pin, &keypad_pin_len_t));
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, "Reset_Pin", &reset_pin, &keypad_pin_len_t));
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, "ESP_AP_PASSWORD", &ESP_AP_PASSWORD, &PW_LENGTH_t));
        } else { // Setup not Complete
            setup_complete = 0;
            ESP_LOGW(TAG, "SETUP NOT COMPLETE");
        }
    }
    configure_peripherals();
}

static void reset_MedSafe(void){
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_erase_all(my_handle));
        setup_complete = 0;
        esp_wifi_disconnect();
        esp_restart();
}

static void reset_pin_buffer(TimerHandle_t xTimer) {
    memset(pin_buffer, 0, sizeof(pin_buffer));
    pin_index = 0;
    ESP_LOGD(TAG_HW, "PIN entry timed out. Buffer reset.");
}

/**
 * TASKS
 */
TaskHandle_t keypad_process_task_h;

static void keypad_read_task(void){
    while (true) {
        uint32_t now = esp_timer_get_time() / 1000;

        if (now - last_key_time > KEYPAD_DEBOUNCING) {
            last_key_time = now;
            for (int row = 0; row < ROWS; row++) {
                gpio_set_level(ROW_PINS[row], 0);

                for (int col = 0; col < COLS; col++) {
                    if (gpio_get_level(COL_PINS[col]) == 0) {
                        char key = KEYPAD_MAP[row][col];
                        xQueueSend(keypad_queue, &key, portMAX_DELAY);
                        vTaskResume(keypad_process_task_h);
                    }
                }

                gpio_set_level(ROW_PINS[row], 1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void keypad_process_task(void){
    char key;

    // Create the timer
    entry_timer = xTimerCreate("EntryTimer", pdMS_TO_TICKS(5000), pdFALSE, NULL, reset_pin_buffer);
    if (entry_timer == NULL) {
        ESP_LOGE(TAG_HW, "Failed to create entry timer.");
    } else {
    // Start the timer
        if (xTimerStart(entry_timer, 0) != pdPASS) {
            ESP_LOGE(TAG_HW, "Failed to start entry timer.");
        }
    }

    while (true)
    {
        if (xQueueReceive(keypad_queue, &key, 50)) {
            ESP_LOGD(TAG, "Key Pressed: %c\n", key);

            // Reset the timer on each keypress
            if (xTimerReset(entry_timer, 0) != pdPASS) {
                ESP_LOGE(TAG_HW, "Failed to reset entry timer.");
            }

            if (key == '#'){
                if(strcmp(pin_buffer, lock_pin) == 0){
                    ESP_LOGD(TAG_HW, "Lock Pin entered. Unlocking...");
                    unlock_container();
                } else if(strcmp(pin_buffer, reset_pin) == 0){
                    ESP_LOGD(TAG_HW, "Reset Pin entered. Resetting...");
                    reset_MedSafe();
                }
                 else {
                    ESP_LOGD(TAG_HW, "Invalid Pin!");
                }
                memset(pin_buffer, 0, sizeof(pin_buffer));
                pin_index = 0;
            } else if (key >= '0' && key <= '9' && pin_index < 4){
                pin_buffer[pin_index] = key;
                pin_buffer[pin_index++] = '\0';
                ESP_LOGD(TAG_HW, "Current Pin: %s", pin_buffer);
            } else {
                ESP_LOGW(TAG_HW, "Invalid Key or buffer full.");
            }
        }
        vTaskSuspend(NULL);
    }
}

static void read_display_status_task(void){
    while (true) {
        read_cooler_temp();
        read_cooler_voltage();
        read_cooler_current();
        check_container_filled();
        check_lid_closed();
        
        ESP_LOGI(TAG_HW, "\nTemp: %f°C\nDuty Cycle: %d\nCurrent: %f A\nVoltage: %f\nContainer: %s\nLid: %s\n", cooler_Temp, duty_cycle,cooler_Current, cooler_Voltage, (container_filled == true) ? "Filled" : "Empty", (lid_closed == true) ? "Closed" : "Open");

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_INTERVAL_MS));
    }
}

static void mqtt_publisher_task(void){
    while (true) {
        if (MQTT_CONNECTED) {
            char payload[64];
            snprintf(payload, sizeof(payload),
                    "{\"temperature\": %.2f, \"container_filled\": %s}",
                    cooler_Temp, container_filled ? "true" : "false");
            // Publish data
            float msg_id = esp_mqtt_client_publish(client, "/Medsafe/data", payload, 0, 0, 0);
            ESP_LOGD(TAG_MQTT, "Published data: %s", payload);
        } else {
            ESP_LOGW(TAG_MQTT, "MQTT not connected, unable to publish");
        }
        
        vTaskDelay(60000 / portTICK_PERIOD_MS); // Publish every minute
    } 
}

static void main_control_task(void){
    while(true){
        if(container_filled && lid_closed){
            if(desired_temp > cooler_Temp){
                lock_container();
                enable_cooler();
                update_duty_cycle(70);
            } else {
                if(duty_cycle != 50){
                    update_duty_cycle(50);
                }
            }
        }else{
            unlock_container();
            disable_cooler();
        }
        vTaskDelay(10);
    }
}

/**
 * MAIN
 */
void app_main(void)
{
    ESP_LOGI(TAG, "");
    
    setup_MedSafe();

    xTaskCreate(read_display_status_task, "update_status_t", 4096, NULL, 5, NULL);
    xTaskCreate(keypad_read_task, "keypad_read_t", 4096, NULL, 3, NULL);
    xTaskCreate(keypad_process_task, "keypad_process_t", 4096, NULL, 4, &keypad_process_task_h);
    xTaskCreate(mqtt_publisher_task, "mqtt_publisher_t", 5120, NULL, 2, NULL);
    xTaskCreate(main_control_task, "main_control_t", 5120, NULL, 8, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}