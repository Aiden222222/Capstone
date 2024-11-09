#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "env_config.h"

static const char *TAG = "ESP32_MQTT";

uint32_t MQTT_CONNECTED = 0;
static esp_mqtt_client_handle_t client = NULL;

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
            msg_id = esp_mqtt_client_subscribe(client, "capstone/topic/test1", 0);
            ESP_LOGI(TAG, "Subscribed to capstone/topic/test1, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            MQTT_CONNECTED = 0;
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
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
    esp_mqtt_client_config_t mqtt_config = {};
    mqtt_config.broker.address.uri = "mqtt://test.mosquitto.org:1883";
    
    client = esp_mqtt_client_init(&mqtt_config);
// Register the event handler for specific MQTT events
    esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_DATA, mqtt_event_handler, client);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ERROR, mqtt_event_handler, client);    esp_mqtt_client_start(client);
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
 * Checks if MQTT is connected before publishing. Sends dummy sensor data to a specified topic
 * every 15 seconds. Logs the result of each publish attempt.
 * 
 * @param params Parameters for the task (not used in this case).
 */

void Publisher_Task(void *params) {
    while (true) {
        if (MQTT_CONNECTED) {
            // Send dummy data
            const char* dummy_data = "Sensor Value: 42"; //Replace with actual sensor data in the future
            int msg_id = esp_mqtt_client_publish(client, "/Capstone_Medical_Device/data", dummy_data, 0, 0, 0);
            // int msg_id = esp_mqtt_client_publish(client, "/topic/test3", "Hello World", 0, 0, 0);
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Message published, msg_id=%d", msg_id);
            } else {
                ESP_LOGE(TAG, "Failed to publish message");
            }

            // Delay to avoid overwhelming the system and reset WDT
            vTaskDelay(15000 / portTICK_PERIOD_MS); // Delay for 15 seconds
        } else {
            ESP_LOGW(TAG, "MQTT not connected, unable to publish");
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait before retrying
        }
    }
}

/**
 * @brief Main entry point of the application.
 * Initializes NVS, Wi-Fi, and starts the publisher task to handle MQTT data publishing.
 */

extern "C" void app_main(void)
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
