idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        driver
        esp_adc
        esp_timer
        esp_event
        esp_netif
        esp_wifi
        esp_http_client
        esp_rom
        freertos
        log
)
