idf_component_register(SRCS "webcam.c" "person_detect.cpp" "gifenc.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server esp_wifi esp_event nvs_flash driver esp_driver_i2s esp32-camera fatfs sdmmc app_update bt spi_flash esp_timer esp_hw_support esp_http_client)
