idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi esp_event nvs_flash protocol_examples_common)