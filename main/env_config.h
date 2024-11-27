#ifndef ENV_CONFIG_H
#define ENV_CONFIG_H

#define MAX_SSID_LENGTH 32
#define MAX_PW_LENGTH 64

char WIFI_SSID[MAX_SSID_LENGTH] = "";
char WIFI_PASSWORD[MAX_PW_LENGTH] = "";
char ESP_AP_PASSWORD[MAX_PW_LENGTH] = "";
#define MQTT_URI "mqtt://test.mosquitto.org:1883"

#define keypad_pin_len 5
char lock_pin[keypad_pin_len] = "";
char reset_pin[keypad_pin_len] = "";


// HTML form to collect user inputs
static const char *html_form =
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <title>MedSafe Web Server</title>\n"
"</head>\n"
"<body>\n"
"    <h1>Device Configuration</h1>\n"
"    <form action=\"/setconfig\" method=\"post\">\n"
"        <label for=\"wifi_ssid\">WiFi SSID:</label><br>\n"
"        <input type=\"text\" id=\"wifi_ssid\" name=\"wifi_ssid\" maxlength=\"32\" required><br><br>\n"
"        <label for=\"wifi_pw\">WiFi Password:</label><br>\n"
"        <input type=\"text\" id=\"wifi_pw\" name=\"wifi_pw\" maxlength=\"64\" required><br><br>\n"
"        <label for=\"reset\">Reset Pin:</label><br>\n"
"        <input type=\"text\" id=\"reset\" name=\"reset\" maxlength=\"4\" required><br><br>\n"
"        <label for=\"lock\">Lock Pin:</label><br>\n"
"        <input type=\"text\" id=\"lock\" name=\"lock\" maxlength=\"4\" required><br><br>\n"
"        <label for=\"ap_pw\">Access Point Password:</label><br>\n"
"        <input type=\"text\" id=\"ap_pw\" name=\"ap_pw\" maxlength=\"64\" required><br><br>\n"
"        <input type=\"submit\" value=\"Save\">\n"
"    </form>\n"
"</body>\n"
"</html>\n";

#endif