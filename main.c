/*
 * Task-Based Weather Monitoring Station
 * Target: Classic ESP32, ESP-IDF 5.x
 *
 * Pin assumptions:
 * DHT11 DATA -> GPIO 4
 * MQ-2 analog -> GPIO 34 (ADC1 channel 6)
 * MQ-135 analog -> GPIO 35 (ADC1 channel 7)
 * SSD1306 OLED SDA -> GPIO 21
 * SSD1306 OLED SCL -> GPIO 22
 *
 * IMPORTANT:
 * - MQ sensor outputs must be scaled so ADC pins never exceed 3.3 V.
 * - MQ readings are raw ADC counts, not calibrated ppm or AQI values.
 * - This is an educational monitor, not a certified safety instrument.
 * - Configure credentials in main/secrets.h (copy secrets.example.h).
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_client.h"

#include "secrets.h"

#define TAG "WEATHER_STATION"
#define NET_TAG "NET_THINGSPEAK"

#define DHT_GPIO GPIO_NUM_4
#define MQ2_ADC_CHANNEL ADC_CHANNEL_6
#define MQ135_ADC_CHANNEL ADC_CHANNEL_7

#define OLED_I2C_PORT I2C_NUM_0
#define OLED_SDA_GPIO GPIO_NUM_21
#define OLED_SCL_GPIO GPIO_NUM_22
#define OLED_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_PAGES 8

#define DHT_PERIOD_MS 2000
#define GAS_PERIOD_MS 1000
#define DISPLAY_PERIOD_MS 1000
#define THINGSPEAK_UPLOAD_PERIOD_MS 20000

#define WIFI_CONNECTED_BIT BIT0

typedef struct {
    float temperature_c;
    float humidity_percent;
    int mq2_raw;
    int mq135_raw;
    bool dht_valid;
    int64_t timestamp_us;
} weather_data_t;

static SemaphoreHandle_t data_mutex;
static EventGroupHandle_t wifi_event_group;
static adc_oneshot_unit_handle_t adc1_handle;

static weather_data_t latest = {
    .temperature_c = 0,
    .humidity_percent = 0,
    .mq2_raw = 0,
    .mq135_raw = 0,
    .dht_valid = false,
    .timestamp_us = 0
};

/*
 * DHT11 pulse capture.
 * Note: this implementation busy-waits in a critical section. It is timing
 * sensitive and should be replaced with a maintained DHT driver/RMT capture
 * for a more robust production design.
 */
static portMUX_TYPE dht_capture_mux = portMUX_INITIALIZER_UNLOCKED;

static bool wait_for_level(int level, uint32_t timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(DHT_GPIO) != level) {
        if ((uint32_t)(esp_timer_get_time() - start) >= timeout_us) {
            return false;
        }
    }
    return true;
}

static bool dht11_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(30);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    int fail_phase = 0;
    int failed_bit = -1;

    portENTER_CRITICAL(&dht_capture_mux);

    if (!wait_for_level(0, 200)) {
        fail_phase = 1;
        goto capture_done;
    }
    if (!wait_for_level(1, 200)) {
        fail_phase = 2;
        goto capture_done;
    }
    if (!wait_for_level(0, 200)) {
        fail_phase = 3;
        goto capture_done;
    }

    for (int bit = 0; bit < 40; bit++) {
        if (!wait_for_level(1, 120)) {
            fail_phase = 4;
            failed_bit = bit;
            goto capture_done;
        }
        int64_t high_start = esp_timer_get_time();
        if (!wait_for_level(0, 120)) {
            fail_phase = 5;
            failed_bit = bit;
            goto capture_done;
        }
        int64_t high_time = esp_timer_get_time() - high_start;
        data[bit / 8] <<= 1;
        if (high_time > 48) {
            data[bit / 8] |= 1;
        }
    }

capture_done:
    portEXIT_CRITICAL(&dht_capture_mux);

    if (fail_phase != 0) {
        ESP_LOGW(TAG, "DHT11 capture failed (phase=%d, bit=%d)",
                 fail_phase, failed_bit);
        return false;
    }

    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        ESP_LOGW(TAG, "DHT11 checksum mismatch");
        return false;
    }

    *humidity = (float)data[0];
    *temperature = (float)(data[2] & 0x7F);
    if (data[2] & 0x80) {
        *temperature = -*temperature;
    }

    if (*humidity > 100.0f || *temperature < 0.0f || *temperature > 50.0f) {
        ESP_LOGW(TAG, "DHT11 value out of expected range");
        return false;
    }
    return true;
}

static void adc_setup(void)
{
    const adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1_handle, MQ2_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1_handle, MQ135_ADC_CHANNEL, &channel_config));
}

static int read_adc_channel(adc_channel_t channel)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return -1;
    }
    return raw;
}

/* Minimal SSD1306 I2C driver and compact 5x7 font. */
static esp_err_t oled_write(bool is_data, const uint8_t *bytes, size_t len)
{
    uint8_t buffer[129];
    if (len > 128) {
        return ESP_ERR_INVALID_SIZE;
    }
    buffer[0] = is_data ? 0x40 : 0x00;
    memcpy(&buffer[1], bytes, len);
    return i2c_master_write_to_device(
        OLED_I2C_PORT, OLED_ADDRESS, buffer, len + 1, pdMS_TO_TICKS(100));
}

static void oled_command(uint8_t cmd)
{
    (void)oled_write(false, &cmd, 1);
}

static void oled_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    ESP_ERROR_CHECK(i2c_param_config(OLED_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(OLED_I2C_PORT, conf.mode, 0, 0, 0));

    const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_command(init_cmds[i]);
    }
}

typedef struct {
    char c;
    uint8_t cols[5];
} glyph_t;

static const glyph_t font[] = {
    {' ',{0,0,0,0,0}}, {'-',{8,8,8,8,8}}, {'.',{0,0x60,0x60,0,0}},
    {':',{0,0x36,0x36,0,0}}, {'0',{0x3E,0x51,0x49,0x45,0x3E}},
    {'1',{0,0x42,0x7F,0x40,0}}, {'2',{0x42,0x61,0x51,0x49,0x46}},
    {'3',{0x21,0x41,0x45,0x4B,0x31}}, {'4',{0x18,0x14,0x12,0x7F,0x10}},
    {'5',{0x27,0x45,0x45,0x45,0x39}}, {'6',{0x3C,0x4A,0x49,0x49,0x30}},
    {'7',{1,0x71,9,5,3}}, {'8',{0x36,0x49,0x49,0x49,0x36}},
    {'9',{6,0x49,0x49,0x29,0x1E}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'C',{0x3E,0x41,0x41,0x41,0x22}},
    {'D',{0x7F,0x41,0x41,0x22,0x1C}}, {'E',{0x7F,0x49,0x49,0x49,0x41}},
    {'G',{0x3E,0x41,0x49,0x49,0x7A}}, {'H',{0x7F,8,8,8,0x7F}},
    {'I',{0,0x41,0x7F,0x41,0}}, {'M',{0x7F,2,0x0C,2,0x7F}},
    {'N',{0x7F,4,8,0x10,0x7F}}, {'O',{0x3E,0x41,0x41,0x41,0x3E}},
    {'P',{0x7F,9,9,9,6}}, {'Q',{0x3E,0x41,0x51,0x21,0x5E}},
    {'R',{0x7F,9,0x19,0x29,0x46}}, {'S',{0x46,0x49,0x49,0x49,0x31}},
    {'T',{1,1,0x7F,1,1}}, {'U',{0x3F,0x40,0x40,0x40,0x3F}},
    {'V',{0x1F,0x20,0x40,0x20,0x1F}}, {'W',{0x3F,0x40,0x38,0x40,0x3F}},
    {'Y',{7,8,0x70,8,7}},
    {'a',{0x20,0x54,0x54,0x54,0x78}}, {'b',{0x7F,0x48,0x44,0x44,0x38}},
    {'c',{0x38,0x44,0x44,0x44,0x20}}, {'d',{0x38,0x44,0x48,0x48,0x7F}},
    {'e',{0x38,0x54,0x54,0x54,0x18}}, {'g',{0x0C,0x52,0x52,0x52,0x3E}},
    {'h',{0x7F,8,4,4,0x78}}, {'i',{0,0x44,0x7D,0x40,0}},
    {'m',{0x7C,4,0x18,4,0x78}}, {'n',{0x7C,8,4,4,0x78}},
    {'o',{0x38,0x44,0x44,0x44,0x38}}, {'p',{0x7C,0x14,0x14,0x14,8}},
    {'r',{0x7C,8,4,4,8}}, {'s',{0x48,0x54,0x54,0x54,0x20}},
    {'t',{4,0x3F,0x44,0x40,0x20}}, {'u',{0x3C,0x40,0x40,0x20,0x7C}},
    {'y',{0x0C,0x50,0x50,0x50,0x3C}}, {'%',{0x63,0x13,8,0x64,0x63}}
};

static const uint8_t *glyph_for(char c)
{
    for (size_t i = 0; i < sizeof(font) / sizeof(font[0]); i++) {
        if (font[i].c == c) {
            return font[i].cols;
        }
    }
    return font[0].cols;
}

static uint8_t frame[OLED_WIDTH * OLED_PAGES];

static void oled_clear(void)
{
    memset(frame, 0, sizeof(frame));
}

static void oled_text(int x, int page, const char *text)
{
    if (page < 0 || page >= OLED_PAGES) {
        return;
    }
    while (*text && x <= OLED_WIDTH - 6) {
        const uint8_t *g = glyph_for(*text++);
        for (int col = 0; col < 5; col++) {
            if (x + col >= 0 && x + col < OLED_WIDTH) {
                frame[page * OLED_WIDTH + x + col] = g[col];
            }
        }
        x += 6;
    }
}

static void oled_refresh(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_command((uint8_t)(0xB0 + page));
        oled_command(0x00);
        oled_command(0x10);
        (void)oled_write(true, &frame[page * OLED_WIDTH], OLED_WIDTH);
    }
}

/* Wi-Fi station setup and event handling. */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(NET_TAG, "Wi-Fi disconnected; reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(NET_TAG, "Wi-Fi connected; IP address received");
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(NET_TAG, "Connecting to configured Wi-Fi network");
}

static void thingspeak_task(void *arg)
{
    (void)arg;
    weather_data_t snapshot;
    char url[384];

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
            pdMS_TO_TICKS(1000));

        if ((bits & WIFI_CONNECTED_BIT) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        memset(&snapshot, 0, sizeof(snapshot));
        if (data_mutex &&
            xSemaphoreTake(data_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            snapshot = latest;
            xSemaphoreGive(data_mutex);
        }

        if (snapshot.dht_valid) {
            snprintf(url, sizeof(url),
                "http://api.thingspeak.com/update?api_key=%s&field1=%.1f&field2=%.1f&field3=%d&field4=%d",
                THINGSPEAK_WRITE_KEY, snapshot.temperature_c,
                snapshot.humidity_percent, snapshot.mq2_raw, snapshot.mq135_raw);
        } else {
            snprintf(url, sizeof(url),
                "http://api.thingspeak.com/update?api_key=%s&field3=%d&field4=%d",
                THINGSPEAK_WRITE_KEY, snapshot.mq2_raw, snapshot.mq135_raw);
        }

        esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_GET,
            .timeout_ms = 10000,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client != NULL) {
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                int status = esp_http_client_get_status_code(client);
                ESP_LOGI(NET_TAG, "ThingSpeak HTTP status=%d", status);
            } else {
                ESP_LOGE(NET_TAG, "ThingSpeak upload failed: %s",
                         esp_err_to_name(err));
            }
            esp_http_client_cleanup(client);
        } else {
            ESP_LOGE(NET_TAG, "Could not create HTTP client");
        }

        vTaskDelay(pdMS_TO_TICKS(THINGSPEAK_UPLOAD_PERIOD_MS));
    }
}

/* FreeRTOS tasks. */
static void dht_task(void *arg)
{
    (void)arg;
    for (;;) {
        float t = 0.0f;
        float h = 0.0f;
        bool ok = dht11_read(&t, &h);

        if (data_mutex &&
            xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (ok) {
                latest.temperature_c = t;
                latest.humidity_percent = h;
                latest.dht_valid = true;
            } else {
                latest.dht_valid = false;
            }
            latest.timestamp_us = esp_timer_get_time();
            xSemaphoreGive(data_mutex);
        }

        ESP_LOGI("SCHEDULER", "DHT task priority=%u status=%s",
                 (unsigned)uxTaskPriorityGet(NULL), ok ? "OK" : "READ_FAILED");
        if (ok) {
            ESP_LOGI(TAG, "DHT11: temperature=%.1f C humidity=%.1f%%", t, h);
        }
        vTaskDelay(pdMS_TO_TICKS(DHT_PERIOD_MS));
    }
}

static void gas_task(void *arg)
{
    (void)arg;
    for (;;) {
        int mq2 = read_adc_channel(MQ2_ADC_CHANNEL);
        int mq135 = read_adc_channel(MQ135_ADC_CHANNEL);

        if (data_mutex &&
            xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            latest.mq2_raw = mq2;
            latest.mq135_raw = mq135;
            latest.timestamp_us = esp_timer_get_time();
            xSemaphoreGive(data_mutex);
        }

        ESP_LOGI("SCHEDULER", "Gas task priority=%u",
                 (unsigned)uxTaskPriorityGet(NULL));
        ESP_LOGI(TAG, "MQ2 raw=%d MQ135 raw=%d", mq2, mq135);
        vTaskDelay(pdMS_TO_TICKS(GAS_PERIOD_MS));
    }
}

static void display_task(void *arg)
{
    (void)arg;
    weather_data_t snapshot = {0};
    char line[24];

    for (;;) {
        if (data_mutex &&
            xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snapshot = latest;
            xSemaphoreGive(data_mutex);
        }

        oled_clear();
        oled_text(0, 0, "Weather Station");

        if (snapshot.dht_valid) {
            snprintf(line, sizeof(line), "Temp: %.1f C", snapshot.temperature_c);
            oled_text(0, 2, line);
            snprintf(line, sizeof(line), "Hum: %.1f %%", snapshot.humidity_percent);
            oled_text(0, 3, line);
        } else {
            oled_text(0, 2, "DHT11: no data");
        }

        snprintf(line, sizeof(line), "MQ2: %d", snapshot.mq2_raw);
        oled_text(0, 5, line);
        snprintf(line, sizeof(line), "MQ135: %d", snapshot.mq135_raw);
        oled_text(0, 6, line);
        oled_refresh();

        ESP_LOGI("SCHEDULER", "Display task priority=%u",
                 (unsigned)uxTaskPriorityGet(NULL));
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Task-Based Weather Monitoring Station");

    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return;
    }

    adc_setup();
    oled_init();
    wifi_init_sta();

    ESP_LOGI("SCHEDULER",
             "Task priorities: DHT=5, Display=4, Gas=3, ThingSpeak=2");

    BaseType_t ok1 = xTaskCreate(dht_task, "dht_task", 4096, NULL, 5, NULL);
    BaseType_t ok2 = xTaskCreate(gas_task, "gas_task", 3072, NULL, 3, NULL);
    BaseType_t ok3 = xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);
    BaseType_t ok4 = xTaskCreate(thingspeak_task, "thingspeak_task", 6144, NULL, 2, NULL);

    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS) {
        ESP_LOGE(TAG, "Failed to create one or more FreeRTOS tasks");
    }
}
