#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "driver/i2s_pdm.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "mdns.h"
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

/* WiFi credentials stored in NVS */
#define NVS_WIFI_NAMESPACE "wifi_cfg"
#define NVS_WIFI_SSID_KEY  "ssid"
#define NVS_WIFI_PASS_KEY  "pass"

/* SoftAP provisioning config */
#define PROV_AP_SSID       "XIAO-SETUP"
#define PROV_AP_PASS       "setup123"
#define PROV_AP_CHANNEL    1
#define PROV_AP_MAX_CONN   4

/* Forward declarations */
static esp_err_t init_camera(void);
static void init_mdns(void);
static esp_err_t init_mic(void);
static esp_err_t audio_handler(httpd_req_t *req);
static esp_err_t ota_handler(httpd_req_t *req);
static esp_err_t config_backup_handler(httpd_req_t *req);
static esp_err_t rec_toggle_handler(httpd_req_t *req);
static esp_err_t rec_list_handler(httpd_req_t *req);
static esp_err_t files_list_handler(httpd_req_t *req);
static esp_err_t download_handler(httpd_req_t *req);
static esp_err_t delete_handler(httpd_req_t *req);
static void rec_enforce_budget(void);
static esp_err_t init_sd_card(void);
static esp_err_t mount_sd_card(bool format_if_mount_failed);
static void wifi_init_sta(void);
static void wifi_init_prov(void);
static esp_err_t wifi_credentials_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
static esp_err_t wifi_credentials_save(const char *ssid, const char *pass);
static httpd_handle_t start_webserver(void);
static httpd_handle_t start_stream_webserver(void);
static httpd_handle_t start_prov_webserver(void);
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t control_handler(httpd_req_t *req);
static esp_err_t reboot_handler(httpd_req_t *req);
static esp_err_t format_sd_handler(httpd_req_t *req);
static esp_err_t prov_index_handler(httpd_req_t *req);
static esp_err_t prov_scan_handler(httpd_req_t *req);
static esp_err_t prov_save_handler(httpd_req_t *req);
static esp_err_t prov_save_get_handler(httpd_req_t *req);
static esp_err_t scan_wifi_networks(void);
static void url_decode(char *dst, const char *src, size_t dst_size);
static void json_escape_string(char *dst, size_t dst_size, const char *src);
static void html_escape_string(char *dst, size_t dst_size, const char *src);
static bool form_get_value(const char *body, const char *key, char *dst, size_t dst_size);
static bool ssid_is_scanned(const char *ssid);
static bool wifi_password_is_valid(const char *pass);
static bool framesize_from_name(const char *name, framesize_t *framesize);
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   10
#define CAM_PIN_SIOD   40
#define CAM_PIN_SIOC   39
#define CAM_PIN_D7     48
#define CAM_PIN_D6     11
#define CAM_PIN_D5     12
#define CAM_PIN_D4     14
#define CAM_PIN_D3     16
#define CAM_PIN_D2     18
#define CAM_PIN_D1     17
#define CAM_PIN_D0     15
#define CAM_PIN_VSYNC  38
#define CAM_PIN_HREF   47
#define CAM_PIN_PCLK   13

/* XIAO ESP32S3 Sense onboard microSD: SDMMC 1-bit mode, no chip-select line */
#define SD_MOUNT_POINT "/sdcard"
#define SD_PIN_CLK     7
#define SD_PIN_CMD     9
#define SD_PIN_D0      8

/* XIAO ESP32S3 Sense onboard PDM microphone (MSM261D3526H1CPM) */
#define MIC_PIN_CLK     42
#define MIC_PIN_DATA    41
#define MIC_SAMPLE_RATE 16000

static const char *TAG = "webcam";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAXIMUM_RETRY 15
#define MAX_SCAN_RESULTS   24

static int wifi_retry_count;
static bool wifi_got_ip_once;
static wifi_ap_record_t scan_results[MAX_SCAN_RESULTS];
static uint16_t scan_result_count;
static bool camera_ready;
static bool sd_ready;
static sdmmc_card_t *sd_card;
static char sd_status_text[96] = "SD not mounted";
static bool mic_ready;
static i2s_chan_handle_t mic_rx_chan;
static SemaphoreHandle_t mic_lock;   /* only one mic consumer (rec vs /audio.wav) at a time */

/* Camera settings persistence */
#define NVS_CAM_NAMESPACE "cam_cfg"

/* Portable config on SD (incl. WiFi creds) — drop-in provisioning + backup */
#define SD_CONFIG_PATH "/sdcard/xiao-config.txt"

/* Loop (dashcam) recording to SD */
#define REC_DIR            "/sdcard/rec"
#define REC_NVS_NS         "rec_cfg"
#define REC_TARGET_FPS     10
#define REC_SEG_MAX_BYTES  (16u * 1024 * 1024)
#define REC_SEG_MAX_FRAMES 2400
static volatile bool rec_enabled;
static bool rec_audio = true;              /* mux audio into clips when mic is present */
static int  rec_budget_pct = 80;           /* keep total clips under this % of the card */
static char rec_status[96] = "idle";

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk  = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href  = CAM_PIN_HREF,
    .pin_pclk  = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* NVS WiFi Credentials */
static esp_err_t wifi_credentials_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    err = nvs_get_str(handle, NVS_WIFI_SSID_KEY, ssid, &ssid_len);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, NVS_WIFI_PASS_KEY, pass, &pass_len);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        if (strlen(ssid) == 0 || !wifi_password_is_valid(pass)) {
            ESP_LOGW(TAG, "Ignoring invalid WiFi credentials in NVS");
            return ESP_ERR_INVALID_STATE;
        }

        for (size_t i = 0; ssid[i]; i++) {
            unsigned char c = (unsigned char)ssid[i];
            if (c < 0x20 || c == 0x7f) {
                ESP_LOGW(TAG, "Ignoring invalid SSID in NVS");
                return ESP_ERR_INVALID_STATE;
            }
        }
    }
    return err;
}

static esp_err_t wifi_credentials_save(const char *ssid, const char *pass) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_str(handle, NVS_WIFI_SSID_KEY, ssid);
    if (err == ESP_OK) err = nvs_set_str(handle, NVS_WIFI_PASS_KEY, pass);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/* Write a portable config (incl. WiFi creds) to the SD card. */
static void sd_config_save(const char *ssid, const char *pass) {
    if (!sd_ready) return;
    FILE *f = fopen(SD_CONFIG_PATH, "w");
    if (!f) {
        ESP_LOGW(TAG, "Could not write %s", SD_CONFIG_PATH);
        return;
    }
    fprintf(f, "# XIAO ESP32S3 camera config\n");
    fprintf(f, "# Drop this SD card into a fresh board to auto-provision WiFi.\n");
    fprintf(f, "ssid=%s\n", ssid);
    fprintf(f, "pass=%s\n", pass);
    fprintf(f, "rec_pct=%d\n", rec_budget_pct);
    fprintf(f, "rec_audio=%d\n", rec_audio ? 1 : 0);
    fclose(f);
    ESP_LOGI(TAG, "Config written to %s", SD_CONFIG_PATH);
}

/* Read config from SD. Returns true if a usable SSID was found; also applies
   any recording settings present. */
static bool sd_config_load(char *ssid, size_t sl, char *pass, size_t pl) {
    FILE *f = fopen(SD_CONFIG_PATH, "r");
    if (!f) return false;
    ssid[0] = '\0';
    pass[0] = '\0';
    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *nl = strpbrk(line, "\r\n");
        if (nl) *nl = '\0';
        if (line[0] == '#' || line[0] == '\0') continue;
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        const char *k = line, *v = eq + 1;
        if (strcmp(k, "ssid") == 0)            strlcpy(ssid, v, sl);
        else if (strcmp(k, "pass") == 0)       strlcpy(pass, v, pl);
        else if (strcmp(k, "rec_pct") == 0)    { int p = atoi(v); if (p >= 10 && p <= 95) rec_budget_pct = p; }
        else if (strcmp(k, "rec_audio") == 0)  rec_audio = atoi(v) ? true : false;
    }
    fclose(f);
    return strlen(ssid) > 0 && wifi_password_is_valid(pass);
}

/* Persist one camera setting (framesize, hmirror, vflip, ...) to NVS. */
static void cam_setting_save(const char *key, int32_t val) {
    nvs_handle_t h;
    if (nvs_open(NVS_CAM_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, key, val);
    nvs_commit(h);
    nvs_close(h);
}

/* Restore saved camera settings so stream and recording use the same look. */
static void cam_settings_load(sensor_t *s) {
    nvs_handle_t h;
    if (nvs_open(NVS_CAM_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "framesize", &v) == ESP_OK)   s->set_framesize(s, (framesize_t)v);
    if (nvs_get_i32(h, "quality", &v) == ESP_OK)     s->set_quality(s, v);
    if (nvs_get_i32(h, "brightness", &v) == ESP_OK)  s->set_brightness(s, v);
    if (nvs_get_i32(h, "contrast", &v) == ESP_OK)    s->set_contrast(s, v);
    if (nvs_get_i32(h, "saturation", &v) == ESP_OK)  s->set_saturation(s, v);
    if (nvs_get_i32(h, "hmirror", &v) == ESP_OK)     s->set_hmirror(s, v);
    if (nvs_get_i32(h, "vflip", &v) == ESP_OK)       s->set_vflip(s, v);
    nvs_close(h);
}

static esp_err_t init_camera(void) {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 12);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    cam_settings_load(s);   /* override defaults with persisted flip/mirror/resolution/etc. */
    camera_ready = true;
    ESP_LOGI(TAG, "Camera initialized");
    return ESP_OK;
}

/* Advertise the board as xiao-cam.local so it can be reached without knowing the IP. */
static void init_mdns(void) {
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set("xiao-cam");
    mdns_instance_name_set("XIAO ESP32S3 Camera");
    mdns_service_add(NULL, "_http._tcp", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS started: http://xiao-cam.local/");
}

static esp_err_t init_mic(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &mic_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mic i2s_new_channel failed: 0x%x", err);
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_PIN_CLK,
            .din = MIC_PIN_DATA,
            .invert_flags = { .clk_inv = false },
        },
    };
    err = i2s_channel_init_pdm_rx_mode(mic_rx_chan, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mic PDM RX init failed: 0x%x", err);
        i2s_del_channel(mic_rx_chan);
        mic_rx_chan = NULL;
        return err;
    }

    err = i2s_channel_enable(mic_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mic i2s enable failed: 0x%x", err);
        i2s_del_channel(mic_rx_chan);
        mic_rx_chan = NULL;
        return err;
    }

    mic_ready = true;
    ESP_LOGI(TAG, "Microphone initialized (PDM mono %d Hz)", MIC_SAMPLE_RATE);
    return ESP_OK;
}

/* Write a 44-byte canonical PCM WAV header for mono 16-bit audio. */
static void wav_header(uint8_t *h, uint32_t sample_rate, uint32_t data_len) {
    uint32_t byte_rate = sample_rate * 2; /* mono, 2 bytes/sample */
    uint32_t riff_len = 36 + data_len;
    memcpy(h, "RIFF", 4);
    h[4] = riff_len; h[5] = riff_len >> 8; h[6] = riff_len >> 16; h[7] = riff_len >> 24;
    memcpy(h + 8, "WAVEfmt ", 8);
    h[16] = 16; h[17] = 0; h[18] = 0; h[19] = 0;   /* fmt chunk size */
    h[20] = 1;  h[21] = 0;                          /* PCM */
    h[22] = 1;  h[23] = 0;                          /* mono */
    h[24] = sample_rate; h[25] = sample_rate >> 8; h[26] = sample_rate >> 16; h[27] = sample_rate >> 24;
    h[28] = byte_rate; h[29] = byte_rate >> 8; h[30] = byte_rate >> 16; h[31] = byte_rate >> 24;
    h[32] = 2; h[33] = 0;                           /* block align */
    h[34] = 16; h[35] = 0;                          /* bits per sample */
    memcpy(h + 36, "data", 4);
    h[40] = data_len; h[41] = data_len >> 8; h[42] = data_len >> 16; h[43] = data_len >> 24;
}

static esp_err_t audio_handler(httpd_req_t *req) {
    if (!mic_ready) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Microphone is not initialized.");
    }
    /* One mic consumer at a time: the recorder may hold it. */
    if (!mic_lock || xSemaphoreTake(mic_lock, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Microphone is busy (recording).");
    }

    int secs = 5;
    int gain = 8;
    char query[48];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char value[8];
        if (httpd_query_key_value(query, "secs", value, sizeof(value)) == ESP_OK) {
            secs = atoi(value);
            if (secs < 1) secs = 1;
            if (secs > 30) secs = 30;
        }
        if (httpd_query_key_value(query, "gain", value, sizeof(value)) == ESP_OK) {
            gain = atoi(value);
            if (gain < 1) gain = 1;
            if (gain > 32) gain = 32;
        }
    }

    uint32_t data_len = (uint32_t)MIC_SAMPLE_RATE * 2 * secs;
    uint8_t header[44];
    wav_header(header, MIC_SAMPLE_RATE, data_len);

    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"xiao-audio.wav\"");
    esp_err_t res = httpd_resp_send_chunk(req, (const char *)header, sizeof(header));

    const size_t buf_bytes = 2048;
    uint8_t *buf = malloc(buf_bytes);
    if (!buf) {
        xSemaphoreGive(mic_lock);
        return ESP_ERR_NO_MEM;
    }

    /* First-order DC blocker (removes the PDM DC bias) + fixed gain. */
    float dc_x = 0.0f, dc_y = 0.0f;
    const float dc_r = 0.995f;

    uint32_t sent = 0;
    while (res == ESP_OK && sent < data_len) {
        size_t want = data_len - sent;
        if (want > buf_bytes) want = buf_bytes;
        size_t got = 0;
        esp_err_t rerr = i2s_channel_read(mic_rx_chan, buf, want, &got, pdMS_TO_TICKS(1000));
        if (rerr != ESP_OK || got == 0) {
            ESP_LOGW(TAG, "Mic read stopped: 0x%x (got %u)", rerr, (unsigned)got);
            break;
        }

        int16_t *samples = (int16_t *)buf;
        size_t count = got / sizeof(int16_t);
        for (size_t i = 0; i < count; i++) {
            float x = samples[i];
            dc_y = x - dc_x + dc_r * dc_y;
            dc_x = x;
            int v = (int)(dc_y * gain);
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            samples[i] = (int16_t)v;
        }

        res = httpd_resp_send_chunk(req, (const char *)buf, count * sizeof(int16_t));
        sent += got;
    }

    free(buf);
    xSemaphoreGive(mic_lock);
    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, NULL, 0);
    }
    return res;
}

/* Push OTA: POST a raw .bin to /ota, written to the inactive OTA slot, then reboot. */
static esp_err_t ota_handler(httpd_req_t *req) {
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }
    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty firmware upload");
        return ESP_FAIL;
    }
    if ((size_t)req->content_len > update->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Firmware larger than OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA start: %d bytes -> partition '%s' (0x%lx)",
             req->content_len, update->label, (unsigned long)update->size);

    esp_ota_handle_t ota;
    esp_err_t err = esp_ota_begin(update, req->content_len, &ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(4096);
    if (!buf) {
        esp_ota_abort(ota);
        return ESP_ERR_NO_MEM;
    }

    int remaining = req->content_len;
    bool ok = true;
    while (remaining > 0) {
        int chunk = remaining > 4096 ? 4096 : remaining;
        int received = httpd_req_recv(req, buf, chunk);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0) {
            ESP_LOGE(TAG, "OTA recv failed: %d", received);
            ok = false;
            break;
        }
        err = esp_ota_write(ota, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            ok = false;
            break;
        }
        remaining -= received;
    }
    free(buf);

    if (!ok) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA transfer failed");
        return ESP_FAIL;
    }

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
            err == ESP_ERR_OTA_VALIDATE_FAILED ? "Image validation failed" : "esp_ota_end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot_partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA success, rebooting into '%s'", update->label);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OTA OK. Rebooting into new firmware...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

/* ===================== Loop recording (AVI: MJPEG video + PCM audio) ===================== */

static void rec_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(REC_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "pct", &v) == ESP_OK && v >= 10 && v <= 95) rec_budget_pct = v;
    if (nvs_get_i32(h, "audio", &v) == ESP_OK) rec_audio = v ? true : false;
    nvs_close(h);
}
static void rec_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(REC_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "pct", rec_budget_pct);
    nvs_set_i32(h, "audio", rec_audio ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

/* Minimal AVI (RIFF) writer. Little-endian target, so plain fwrite of u32 is fine. */
typedef struct { uint32_t off, size; uint8_t audio; } rec_idx_t;
typedef struct {
    FILE *f;
    bool audio;
    uint32_t vframes, abytes, movi_bytes;
    long p_riff, p_totframes, p_vidlen, p_audlen, p_movisz, movi_ref;
    rec_idx_t *idx; uint32_t idx_cap, idx_len;
} avi_t;

static void w_u32(FILE *f, uint32_t v) { uint8_t b[4] = {v, v >> 8, v >> 16, v >> 24}; fwrite(b, 1, 4, f); }
static void w_u16(FILE *f, uint16_t v) { uint8_t b[2] = {v, v >> 8}; fwrite(b, 1, 2, f); }
static void w_tag(FILE *f, const char *t) { fwrite(t, 1, 4, f); }
static void patch_u32(FILE *f, long pos, uint32_t v) { long cur = ftell(f); fseek(f, pos, SEEK_SET); w_u32(f, v); fseek(f, cur, SEEK_SET); }

static bool avi_begin(avi_t *w, const char *path, int width, int height, int fps, bool audio, int sr) {
    memset(w, 0, sizeof(*w));
    w->f = fopen(path, "wb");
    if (!w->f) return false;
    w->audio = audio;
    w->idx_cap = 512;
    w->idx = heap_caps_malloc(w->idx_cap * sizeof(rec_idx_t), MALLOC_CAP_SPIRAM);
    if (!w->idx) w->idx = malloc(w->idx_cap * sizeof(rec_idx_t));
    if (!w->idx) { fclose(w->f); w->f = NULL; return false; }
    FILE *f = w->f;
    uint32_t hdrl = audio ? 292 : 192;
    w_tag(f, "RIFF"); w->p_riff = ftell(f); w_u32(f, 0); w_tag(f, "AVI ");
    w_tag(f, "LIST"); w_u32(f, hdrl); w_tag(f, "hdrl");
    w_tag(f, "avih"); w_u32(f, 56);
    w_u32(f, fps > 0 ? 1000000 / fps : 100000); w_u32(f, 0); w_u32(f, 0); w_u32(f, 0x10);
    w->p_totframes = ftell(f); w_u32(f, 0);
    w_u32(f, 0); w_u32(f, audio ? 2 : 1); w_u32(f, 0);
    w_u32(f, width); w_u32(f, height); w_u32(f, 0); w_u32(f, 0); w_u32(f, 0); w_u32(f, 0);
    /* video stream */
    w_tag(f, "LIST"); w_u32(f, 116); w_tag(f, "strl");
    w_tag(f, "strh"); w_u32(f, 56);
    w_tag(f, "vids"); w_tag(f, "MJPG"); w_u32(f, 0); w_u16(f, 0); w_u16(f, 0);
    w_u32(f, 0); w_u32(f, 1); w_u32(f, fps > 0 ? fps : 10); w_u32(f, 0);
    w->p_vidlen = ftell(f); w_u32(f, 0);
    w_u32(f, 0); w_u32(f, 0xFFFFFFFF); w_u32(f, 0);
    w_u16(f, 0); w_u16(f, 0); w_u16(f, width); w_u16(f, height);
    w_tag(f, "strf"); w_u32(f, 40);
    w_u32(f, 40); w_u32(f, width); w_u32(f, height); w_u16(f, 1); w_u16(f, 24);
    w_tag(f, "MJPG"); w_u32(f, width * height * 3); w_u32(f, 0); w_u32(f, 0); w_u32(f, 0); w_u32(f, 0);
    /* audio stream */
    if (audio) {
        w_tag(f, "LIST"); w_u32(f, 92); w_tag(f, "strl");
        w_tag(f, "strh"); w_u32(f, 56);
        w_tag(f, "auds"); w_u32(f, 0); w_u32(f, 0); w_u16(f, 0); w_u16(f, 0);
        w_u32(f, 0); w_u32(f, 1); w_u32(f, sr); w_u32(f, 0);
        w->p_audlen = ftell(f); w_u32(f, 0);
        w_u32(f, 0); w_u32(f, 0xFFFFFFFF); w_u32(f, 2);
        w_u16(f, 0); w_u16(f, 0); w_u16(f, 0); w_u16(f, 0);
        w_tag(f, "strf"); w_u32(f, 16);
        w_u16(f, 1); w_u16(f, 1); w_u32(f, sr); w_u32(f, sr * 2); w_u16(f, 2); w_u16(f, 16);
    }
    w_tag(f, "LIST"); w->p_movisz = ftell(f); w_u32(f, 0); w->movi_ref = ftell(f); w_tag(f, "movi");
    return true;
}

static void avi_chunk(avi_t *w, const char *fourcc, const uint8_t *data, uint32_t len, uint8_t is_audio) {
    if (!w->f) return;
    if (w->idx_len >= w->idx_cap) {
        uint32_t nc = w->idx_cap * 2;
        rec_idx_t *ni = heap_caps_realloc(w->idx, nc * sizeof(rec_idx_t), MALLOC_CAP_SPIRAM);
        if (!ni) ni = realloc(w->idx, nc * sizeof(rec_idx_t));
        if (!ni) return;
        w->idx = ni; w->idx_cap = nc;
    }
    long pos = ftell(w->f);
    w->idx[w->idx_len].off = (uint32_t)(pos - w->movi_ref);
    w->idx[w->idx_len].size = len;
    w->idx[w->idx_len].audio = is_audio;
    w->idx_len++;
    w_tag(w->f, fourcc); w_u32(w->f, len);
    fwrite(data, 1, len, w->f);
    uint32_t total = 8 + len;
    if (len & 1) { uint8_t z = 0; fwrite(&z, 1, 1, w->f); total++; }
    w->movi_bytes += total;
    if (is_audio) w->abytes += len; else w->vframes++;
}
static void avi_add_video(avi_t *w, const uint8_t *j, uint32_t n) { avi_chunk(w, "00dc", j, n, 0); }
static void avi_add_audio(avi_t *w, const uint8_t *p, uint32_t n) { avi_chunk(w, "01wb", p, n, 1); }

static void avi_end(avi_t *w) {
    if (!w->f) return;
    FILE *f = w->f;
    w_tag(f, "idx1"); w_u32(f, w->idx_len * 16);
    for (uint32_t i = 0; i < w->idx_len; i++) {
        w_tag(f, w->idx[i].audio ? "01wb" : "00dc");
        w_u32(f, 0x10); w_u32(f, w->idx[i].off); w_u32(f, w->idx[i].size);
    }
    long filesize = ftell(f);
    patch_u32(f, w->p_riff, (uint32_t)(filesize - 8));
    patch_u32(f, w->p_totframes, w->vframes);
    patch_u32(f, w->p_vidlen, w->vframes);
    if (w->audio) patch_u32(f, w->p_audlen, w->abytes / 2);
    patch_u32(f, w->p_movisz, 4 + w->movi_bytes);
    fclose(f);
    w->f = NULL;
    free(w->idx); w->idx = NULL;
}

static int rec_seq = -1;
static void rec_next_path(char *out, size_t n) {
    if (rec_seq < 0) {
        rec_seq = 0;
        DIR *d = opendir(REC_DIR);
        if (d) {
            struct dirent *e;
            while ((e = readdir(d))) { int x; if (sscanf(e->d_name, "clip%d.avi", &x) == 1 && x >= rec_seq) rec_seq = x + 1; }
            closedir(d);
        }
    }
    snprintf(out, n, REC_DIR "/clip%05d.avi", rec_seq++);
}

/* Delete oldest clips until total recorded size fits under the budget (% of card). */
static void rec_enforce_budget(void) {
    if (!sd_ready || !sd_card) return;
    static char names[256][32];
    static uint64_t sizes[256];
    uint64_t cap = (uint64_t)sd_card->csd.capacity * sd_card->csd.sector_size;
    uint64_t budget = cap / 100 * rec_budget_pct;
    int n = 0; uint64_t total = 0;
    DIR *d = opendir(REC_DIR);
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) && n < 256) {
        if (strncmp(e->d_name, "clip", 4) != 0) continue;
        char full[300]; snprintf(full, sizeof(full), REC_DIR "/%s", e->d_name);
        struct stat st; if (stat(full, &st) != 0) continue;
        strlcpy(names[n], e->d_name, sizeof(names[n])); sizes[n] = st.st_size; total += st.st_size; n++;
    }
    closedir(d);
    while (total > budget && n > 0) {
        int oldest = 0;
        for (int i = 1; i < n; i++) if (strcmp(names[i], names[oldest]) < 0) oldest = i;
        char full[300]; snprintf(full, sizeof(full), REC_DIR "/%s", names[oldest]);
        unlink(full);
        ESP_LOGI(TAG, "rec budget: deleted %s", names[oldest]);
        total -= sizes[oldest];
        n--;
        if (oldest != n) { strlcpy(names[oldest], names[n], sizeof(names[oldest])); sizes[oldest] = sizes[n]; }
    }
}

static void rec_task(void *arg) {
    avi_t w; bool open = false, have_mic = false; char cur[32] = "";
    int16_t *apcm = NULL;
    const int aframe = MIC_SAMPLE_RATE / REC_TARGET_FPS;
    float dcx = 0, dcy = 0;
    while (1) {
        if (!rec_enabled || !sd_ready || !camera_ready) {
            if (open) { avi_end(&w); open = false; rec_enforce_budget(); }
            if (have_mic) { xSemaphoreGive(mic_lock); have_mic = false; }
            snprintf(rec_status, sizeof(rec_status), "%s", !sd_ready ? "idle (no SD)" : "idle");
            vTaskDelay(pdMS_TO_TICKS(400));
            continue;
        }
        bool want_audio = rec_audio && mic_ready;
        if (want_audio && !have_mic) { if (xSemaphoreTake(mic_lock, 0) == pdTRUE) have_mic = true; }
        if (!want_audio && have_mic) { xSemaphoreGive(mic_lock); have_mic = false; }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (!open) {
            mkdir(REC_DIR, 0777);
            char path[80]; rec_next_path(path, sizeof(path));
            const char *bn = strrchr(path, '/'); strlcpy(cur, bn ? bn + 1 : path, sizeof(cur));
            if (avi_begin(&w, path, fb->width, fb->height, REC_TARGET_FPS, have_mic, MIC_SAMPLE_RATE)) {
                open = true;
            } else {
                esp_camera_fb_return(fb);
                snprintf(rec_status, sizeof(rec_status), "error: cannot create clip");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }
        avi_add_video(&w, fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (have_mic) {
            if (!apcm) apcm = malloc(aframe * 2);
            size_t got = 0;
            if (apcm) i2s_channel_read(mic_rx_chan, apcm, aframe * 2, &got, pdMS_TO_TICKS(150));
            if (got >= 2) {
                size_t cnt = got / 2;
                for (size_t i = 0; i < cnt; i++) {
                    float x = apcm[i]; dcy = x - dcx + 0.995f * dcy; dcx = x;
                    int v = (int)(dcy * 8);
                    if (v > 32767) v = 32767; else if (v < -32768) v = -32768;
                    apcm[i] = (int16_t)v;
                }
                avi_add_audio(&w, (uint8_t *)apcm, cnt * 2);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000 / REC_TARGET_FPS));
        }

        snprintf(rec_status, sizeof(rec_status), "REC %s (%u frames%s)", cur,
                 (unsigned)w.vframes, have_mic ? "+audio" : "");
        if (w.movi_bytes >= REC_SEG_MAX_BYTES || w.vframes >= REC_SEG_MAX_FRAMES) {
            avi_end(&w); open = false; rec_enforce_budget();
        }
    }
}

static esp_err_t rec_toggle_handler(httpd_req_t *req) {
    char q[64], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "on", v, sizeof(v)) == ESP_OK) rec_enabled = atoi(v) ? true : false;
        if (httpd_query_key_value(q, "pct", v, sizeof(v)) == ESP_OK) { int p = atoi(v); if (p >= 10 && p <= 95) { rec_budget_pct = p; rec_cfg_save(); } }
        if (httpd_query_key_value(q, "audio", v, sizeof(v)) == ESP_OK) { rec_audio = atoi(v) ? true : false; rec_cfg_save(); }
    }
    char body[128];
    snprintf(body, sizeof(body), "{\"enabled\":%s,\"audio\":%s,\"pct\":%d}",
             rec_enabled ? "true" : "false", rec_audio ? "true" : "false", rec_budget_pct);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t rec_list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    uint64_t total = (sd_ready && sd_card) ? (uint64_t)sd_card->csd.capacity * sd_card->csd.sector_size : 0;
    uint64_t used = 0;
    httpd_resp_sendstr_chunk(req, "{\"clips\":[");
    DIR *d = opendir(REC_DIR);
    bool first = true;
    if (d) {
        struct dirent *e;
        while ((e = readdir(d))) {
            if (strncmp(e->d_name, "clip", 4) != 0) continue;
            char full[300]; snprintf(full, sizeof(full), REC_DIR "/%s", e->d_name);
            struct stat st; if (stat(full, &st) != 0) continue;
            used += st.st_size;
            char item[320];
            snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"size\":%lld}", first ? "" : ",", e->d_name, (long long)st.st_size);
            first = false;
            httpd_resp_sendstr_chunk(req, item);
        }
        closedir(d);
    }
    char tail[224];
    snprintf(tail, sizeof(tail), "],\"enabled\":%s,\"audio\":%s,\"pct\":%d,\"used\":%llu,\"total\":%llu,\"status\":\"%s\"}",
             rec_enabled ? "true" : "false", rec_audio ? "true" : "false", rec_budget_pct,
             (unsigned long long)used, (unsigned long long)total, rec_status);
    httpd_resp_sendstr_chunk(req, tail);
    return httpd_resp_sendstr_chunk(req, NULL);
}

/* --- Generic SD file browser (confined to /sdcard) --- */
static bool sd_path_ok(const char *p) {
    return p && strncmp(p, "/sdcard", 7) == 0 && !strstr(p, "..");
}

static esp_err_t files_list_handler(httpd_req_t *req) {
    char q[192], dir[128] = "/sdcard";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[128]; if (httpd_query_key_value(q, "dir", v, sizeof(v)) == ESP_OK) url_decode(dir, v, sizeof(dir));
    }
    if (!sd_path_ok(dir)) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path"); return ESP_FAIL; }
    DIR *d = opendir(dir);
    if (!d) { httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such dir"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    char dir_esc[192], open_json[224];
    json_escape_string(dir_esc, sizeof(dir_esc), dir);
    snprintf(open_json, sizeof(open_json), "{\"dir\":\"%s\",\"entries\":[", dir_esc);
    httpd_resp_sendstr_chunk(req, open_json);
    struct dirent *e; bool first = true;
    while ((e = readdir(d))) {
        char full[400]; snprintf(full, sizeof(full), "%s/%s", dir, e->d_name);
        struct stat st; if (stat(full, &st) != 0) continue;
        char esc[128]; json_escape_string(esc, sizeof(esc), e->d_name);
        char item[256];
        snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"dir\":%s,\"size\":%lld}",
                 first ? "" : ",", esc, S_ISDIR(st.st_mode) ? "true" : "false", (long long)st.st_size);
        first = false;
        httpd_resp_sendstr_chunk(req, item);
    }
    closedir(d);
    httpd_resp_sendstr_chunk(req, "]}");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t download_handler(httpd_req_t *req) {
    char q[224], path[160] = "", v[160];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "path", v, sizeof(v)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no path"); return ESP_FAIL;
    }
    url_decode(path, v, sizeof(path));
    if (!sd_path_ok(path)) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path"); return ESP_FAIL; }
    FILE *f = fopen(path, "rb");
    if (!f) { httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found"); return ESP_FAIL; }
    const char *bn = strrchr(path, '/'); bn = bn ? bn + 1 : path;
    httpd_resp_set_type(req, strstr(path, ".avi") ? "video/x-msvideo" :
                             strstr(path, ".wav") ? "audio/wav" : "application/octet-stream");
    char cd[300]; snprintf(cd, sizeof(cd), "attachment; filename=\"%s\"", bn);
    httpd_resp_set_hdr(req, "Content-Disposition", cd);
    char *buf = malloc(4096);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t r; esp_err_t res = ESP_OK;
    while ((r = fread(buf, 1, 4096, f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, r) != ESP_OK) { res = ESP_FAIL; break; }
    }
    free(buf); fclose(f);
    if (res == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
    return res;
}

static esp_err_t delete_handler(httpd_req_t *req) {
    char q[224], path[160] = "", v[160];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "path", v, sizeof(v)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no path"); return ESP_FAIL;
    }
    url_decode(path, v, sizeof(path));
    if (!sd_path_ok(path)) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path"); return ESP_FAIL; }
    if (unlink(path) != 0) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "delete failed"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* Back up the current config (incl. WiFi creds from NVS) to the SD card. */
static esp_err_t config_backup_handler(httpd_req_t *req) {
    if (!sd_ready) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No SD card mounted"); return ESP_FAIL; }
    char ssid[32] = {0}, pass[64] = {0};
    if (wifi_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)) != ESP_OK || !ssid[0]) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No saved WiFi to back up");
        return ESP_FAIL;
    }
    sd_config_save(ssid, pass);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Config saved to " SD_CONFIG_PATH);
}

static esp_err_t init_sd_card(void) {
    return mount_sd_card(false);
}

static esp_err_t mount_sd_card(bool format_if_mount_failed) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);
    if (err != ESP_OK) {
        snprintf(sd_status_text, sizeof(sd_status_text), "SD not mounted: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "%s", sd_status_text);
        return err;
    }

    sd_ready = true;
    uint64_t total_mb = ((uint64_t)sd_card->csd.capacity * sd_card->csd.sector_size) / (1024 * 1024);
    snprintf(sd_status_text, sizeof(sd_status_text), "SD mounted: %llu MB total",
             (unsigned long long)total_mb);
    ESP_LOGI(TAG, "%s", sd_status_text);
    sdmmc_card_print_info(stdout, sd_card);
    return ESP_OK;
}

static void wifi_event_handler_sta(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        if (wifi_got_ip_once) {
            /* An established link dropped (e.g. router reboot / signal loss):
               reconnect forever with a short backoff so the board self-heals. */
            ESP_LOGW(TAG, "WiFi lost, reconnecting...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_wifi_connect();
        } else if (wifi_retry_count < WIFI_MAXIMUM_RETRY) {
            wifi_retry_count++;
            ESP_LOGI(TAG, "WiFi disconnected, retrying (%d/%d)...",
                     wifi_retry_count, WIFI_MAXIMUM_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "WiFi connection failed, switching to provisioning mode");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        wifi_got_ip_once = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_event_handler_ap(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_sta(void) {
    char ssid[32] = {0};
    char pass[64] = {0};

    if (wifi_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)) != ESP_OK || strlen(ssid) == 0) {
        /* Before provisioning, try a config file on the SD card (drop-in setup). */
        if (sd_config_load(ssid, sizeof(ssid), pass, sizeof(pass))) {
            ESP_LOGI(TAG, "Loaded WiFi credentials from SD (%s); persisting to NVS", SD_CONFIG_PATH);
            wifi_credentials_save(ssid, pass);
        } else {
            ESP_LOGW(TAG, "No WiFi credentials in NVS or SD, starting provisioning mode");
            wifi_init_prov();
            return;
        }
    }

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();  // needed for potential AP fallback

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = strlen(pass) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_LOGI(TAG, "Loaded WiFi credentials for SSID: %s", ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Always-on camera on mains power: disable modem sleep. v6.0 defaults to
       WIFI_PS_MIN_MODEM, which adds latency and packet loss for a server role. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi STA started, connecting to %s...", ssid);
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        return;
    }

    ESP_LOGW(TAG, "STA connection timed out or failed, starting provisioning mode");
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    scan_wifi_networks();
    ESP_ERROR_CHECK(esp_wifi_stop());

    wifi_config_t ap_config = {
        .ap = {
            .ssid = PROV_AP_SSID,
            .ssid_len = strlen(PROV_AP_SSID),
            .channel = PROV_AP_CHANNEL,
            .password = PROV_AP_PASS,
            .max_connection = PROV_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    if (strlen(PROV_AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_ap, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Provisioning AP started: SSID=%s, PASS=%s, IP=192.168.4.1",
             PROV_AP_SSID, PROV_AP_PASS);
    start_prov_webserver();
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static void wifi_init_prov(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    scan_wifi_networks();
    ESP_ERROR_CHECK(esp_wifi_stop());

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = PROV_AP_SSID,
            .ssid_len = strlen(PROV_AP_SSID),
            .channel = PROV_AP_CHANNEL,
            .password = PROV_AP_PASS,
            .max_connection = PROV_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    if (strlen(PROV_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_ap, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Provisioning AP started: SSID=%s, PASS=%s, IP=192.168.4.1", PROV_AP_SSID, PROV_AP_PASS);
    start_prov_webserver();
    
    // Block forever - provisioning web server handles config save + reboot
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static const char *STREAM_BOUNDARY = "frame";
static const char *STREAM_PART = "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";

#define STREAM_MAX_CLIENTS 3
static QueueHandle_t stream_queue;
static SemaphoreHandle_t stream_slots;

static esp_err_t do_stream(httpd_req_t *req) {
    if (!camera_ready) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Camera is not initialized. Check the camera module and reboot.");
    }

    esp_err_t res = ESP_OK;
    char *part_buf = malloc(128);
    if (!part_buf) return ESP_ERR_NO_MEM;

    res = httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    if (res != ESP_OK) { free(part_buf); return res; }
    res = httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (res != ESP_OK) { free(part_buf); return res; }
    res = httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    if (res != ESP_OK) { free(part_buf); return res; }
    res = httpd_resp_set_hdr(req, "Pragma", "no-cache");
    if (res != ESP_OK) { free(part_buf); return res; }

    int sock = httpd_req_to_sockfd(req);
    /* Bound how long a stalled/gone client can block a send, so this handler
       exits promptly and frees the single stream worker for the next viewer. */
    struct timeval send_to = { .tv_sec = 4, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_to, sizeof(send_to));

    while (1) {
        // Non-blocking client-disconnect check (does not throttle frame rate)
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);
        struct timeval no_wait = { .tv_sec = 0, .tv_usec = 0 };
        int sel = select(sock + 1, &rfds, NULL, NULL, &no_wait);
        if (sel > 0 && FD_ISSET(sock, &rfds)) {
            char dummy[1];
            int r = recv(sock, dummy, 1, MSG_PEEK | MSG_DONTWAIT);
            if (r == 0) {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int len = snprintf(part_buf, 128, STREAM_PART, STREAM_BOUNDARY, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, len);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;

        // Frame rate limiting ~30 FPS
        vTaskDelay(pdMS_TO_TICKS(33));
    }
    free(part_buf);
    return res;
}

/* Each worker owns one concurrent MJPEG client. The httpd server task hands
   off the (async-copied) request and is immediately free to accept others. */
static void stream_worker(void *arg) {
    httpd_req_t *req;
    while (1) {
        if (xQueueReceive(stream_queue, &req, portMAX_DELAY) == pdTRUE) {
            do_stream(req);
            httpd_req_async_handler_complete(req);
            xSemaphoreGive(stream_slots);
        }
    }
}

static esp_err_t stream_handler(httpd_req_t *req) {
    if (!camera_ready) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Camera is not initialized. Check the camera module and reboot.");
    }

    if (xSemaphoreTake(stream_slots, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Too many stream viewers, try again shortly.");
    }

    httpd_req_t *copy = NULL;
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) {
        xSemaphoreGive(stream_slots);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream dispatch failed");
        return ESP_FAIL;
    }
    if (xQueueSend(stream_queue, &copy, 0) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        xSemaphoreGive(stream_slots);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream queue full");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t control_handler(httpd_req_t *req) {
    if (!camera_ready) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Camera is not initialized");
        return ESP_FAIL;
    }

    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Camera sensor unavailable");
        return ESP_FAIL;
    }

    char value[32];
    esp_err_t err = ESP_OK;

    if (httpd_query_key_value(query, "framesize", value, sizeof(value)) == ESP_OK) {
        framesize_t fs;
        if (!framesize_from_name(value, &fs)) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid framesize");
            return ESP_FAIL;
        }
        err = s->set_framesize(s, fs);
        if (err == ESP_OK) cam_setting_save("framesize", fs);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "quality", value, sizeof(value)) == ESP_OK) {
        int quality = atoi(value);
        if (quality < 4 || quality > 30) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid quality");
            return ESP_FAIL;
        }
        err = s->set_quality(s, quality);
        if (err == ESP_OK) cam_setting_save("quality", quality);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "brightness", value, sizeof(value)) == ESP_OK) {
        int level = atoi(value);
        if (level < -2 || level > 2) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid brightness");
            return ESP_FAIL;
        }
        err = s->set_brightness(s, level);
        if (err == ESP_OK) cam_setting_save("brightness", level);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "contrast", value, sizeof(value)) == ESP_OK) {
        int level = atoi(value);
        if (level < -2 || level > 2) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid contrast");
            return ESP_FAIL;
        }
        err = s->set_contrast(s, level);
        if (err == ESP_OK) cam_setting_save("contrast", level);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "saturation", value, sizeof(value)) == ESP_OK) {
        int level = atoi(value);
        if (level < -2 || level > 2) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid saturation");
            return ESP_FAIL;
        }
        err = s->set_saturation(s, level);
        if (err == ESP_OK) cam_setting_save("saturation", level);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "hmirror", value, sizeof(value)) == ESP_OK) {
        int on = atoi(value) ? 1 : 0;
        err = s->set_hmirror(s, on);
        if (err == ESP_OK) cam_setting_save("hmirror", on);
    }
    if (err == ESP_OK && httpd_query_key_value(query, "vflip", value, sizeof(value)) == ESP_OK) {
        int on = atoi(value) ? 1 : 0;
        err = s->set_vflip(s, on);
        if (err == ESP_OK) cam_setting_save("vflip", on);
    }

    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera control failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t reboot_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Rebooting");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t format_sd_handler(httpd_req_t *req) {
    if (sd_ready && sd_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        sd_ready = false;
        sd_card = NULL;
        strlcpy(sd_status_text, "SD unmounted", sizeof(sd_status_text));
    }

    esp_err_t err = mount_sd_card(true);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, sd_status_text);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, sd_status_text);
}

static esp_err_t index_handler(httpd_req_t *req) {
    char sd_status_html[sizeof(sd_status_text) * 6 + 1];
    html_escape_string(sd_status_html, sizeof(sd_status_html), sd_status_text);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'><title>XIAO Camera</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        ":root{color-scheme:dark;--bg:#0a0d10;--panel:#141b22;--panel2:#1a232c;--line:#243039;--text:#e8eef2;--muted:#8b99a6;--accent:#19c3ae;--accent-d:#12a08e;--danger:#e5484d;--r:12px;}"
        "*{box-sizing:border-box}body{margin:0;color:var(--text);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;-webkit-font-smoothing:antialiased;background:radial-gradient(1200px 600px at 50% -10%,#12202a,var(--bg)) fixed;}"
        "header{position:sticky;top:0;z-index:5;height:60px;display:flex;align-items:center;justify-content:space-between;padding:0 20px;border-bottom:1px solid var(--line);background:rgba(15,20,26,.82);backdrop-filter:blur(10px);}"
        ".brand{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.2px}.brand .logo{width:26px;height:26px;border-radius:7px;background:linear-gradient(135deg,var(--accent),#0e7d94);display:grid;place-items:center;font-size:13px;color:#052723}"
        ".status{display:flex;align-items:center;gap:8px;color:var(--muted);font-size:13px}"
        ".dot{width:8px;height:8px;border-radius:50%;background:var(--accent);animation:pulse 2s infinite}.dot.off{background:var(--muted);animation:none}"
        "@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(25,195,174,.5)}70%{box-shadow:0 0 0 7px rgba(25,195,174,0)}100%{box-shadow:0 0 0 0 rgba(25,195,174,0)}}"
        "main{max-width:980px;margin:0 auto;padding:22px 16px 42px}"
        ".frame{position:relative;background:#000;border:1px solid var(--line);border-radius:var(--r);overflow:hidden;box-shadow:0 24px 60px rgba(0,0,0,.45)}"
        ".frame img{display:block;width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;background:#000}"
        ".badges{position:absolute;top:12px;left:12px;right:12px;display:flex;justify-content:space-between;pointer-events:none}"
        ".badge{background:rgba(0,0,0,.55);backdrop-filter:blur(6px);border:1px solid rgba(255,255,255,.14);color:#fff;font-size:12px;font-weight:600;padding:5px 11px;border-radius:999px}"
        ".badge.live{color:var(--accent)}.badge.live::before{content:'';display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--accent);margin-right:6px;vertical-align:middle;animation:pulse 2s infinite}"
        ".offline{aspect-ratio:4/3;display:grid;place-content:center;text-align:center;color:var(--muted);padding:24px;gap:6px}.offline h2{margin:0;color:var(--text);font-size:20px}.offline p{margin:0}"
        ".toolbar{display:flex;gap:10px;margin:14px 0;flex-wrap:wrap}"
        ".btn{flex:1;min-width:120px;height:44px;border:1px solid var(--line);border-radius:10px;background:var(--panel2);color:var(--text);font-weight:600;font-size:14px;cursor:pointer;display:inline-flex;align-items:center;justify-content:center;gap:8px;text-decoration:none;transition:.15s}"
        ".btn:hover{border-color:var(--accent);background:#1e2a33}.btn.primary{background:linear-gradient(135deg,var(--accent),var(--accent-d));border:0;color:#052723}.btn.danger:hover{border-color:var(--danger);color:#ff8b8e}"
        ".card{background:var(--panel);border:1px solid var(--line);border-radius:var(--r);padding:18px;margin-top:14px}.card h3{margin:0 0 14px;font-size:12px;text-transform:uppercase;letter-spacing:.9px;color:var(--muted)}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:16px 22px}"
        ".ctl label{display:block;font-size:13px;color:var(--muted);margin:0 0 8px}.ctl .row{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}.ctl .row label{margin:0}.ctl .val{font-variant-numeric:tabular-nums;color:var(--text);font-weight:600;font-size:13px}"
        "select{width:100%;height:40px;border:1px solid var(--line);border-radius:8px;background:var(--panel2);color:var(--text);padding:0 10px;font-size:14px}"
        "input[type=range]{-webkit-appearance:none;width:100%;height:6px;border-radius:999px;background:var(--line);outline:none}input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;border-radius:50%;background:var(--accent);cursor:pointer;box-shadow:0 2px 6px rgba(0,0,0,.4)}"
        ".ctl.switch{display:flex;align-items:center;justify-content:space-between}.ctl.switch label{margin:0}"
        ".tgl{position:relative;width:46px;height:26px;flex:none}.tgl input{opacity:0;width:0;height:0}.tgl .sl{position:absolute;inset:0;background:var(--line);border-radius:999px;transition:.2s;cursor:pointer}.tgl .sl::before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.2s}.tgl input:checked+.sl{background:var(--accent)}.tgl input:checked+.sl::before{transform:translateX(20px)}"
        ".meta{display:flex;justify-content:space-between;gap:12px;color:var(--muted);font-size:13px;margin:14px 2px 0}"
        "@media(max-width:620px){.grid{grid-template-columns:1fr}.status .txt{display:none}main{padding:16px 12px 32px}.meta{flex-direction:column;gap:4px}}"
        "</style></head><body>"
        "<header><div class='brand'><span class='logo'>&#9673;</span>XIAO ESP32S3 Sense</div>"
        "<div class='status'><span class='dot");
    httpd_resp_sendstr_chunk(req, camera_ready
        ? "'></span><span class='txt'>Live MJPEG"
        : " off'></span><span class='txt'>Camera offline");
    httpd_resp_sendstr_chunk(req, "</span></div></header><main>");

    if (camera_ready) {
        httpd_resp_sendstr_chunk(req,
            "<div class='frame'><div class='badges'><span class='badge live'>LIVE</span>"
            "<span class='badge' id='res'>VGA 640×480</span></div>"
            "<img id='cam' crossorigin='anonymous' alt='Camera stream'></div>"
            "<div class='toolbar'>"
            "<button class='btn primary' id='snap' type='button'>Snapshot</button>"
            "<button class='btn' id='full' type='button'>Fullscreen</button>"
            "<a class='btn' id='raw' href='#' target='_blank' rel='noopener'>Raw stream</a>"
            "</div>"
            "<section class='card'><h3>Camera</h3><div class='grid'>"
            "<div class='ctl'><label for='framesize'>Resolution</label><select id='framesize'>"
            "<option value='QVGA' data-dim='QVGA 320×240'>QVGA · 320×240</option>"
            "<option value='VGA' data-dim='VGA 640×480' selected>VGA · 640×480</option>"
            "<option value='SVGA' data-dim='SVGA 800×600'>SVGA · 800×600</option>"
            "<option value='XGA' data-dim='XGA 1024×768'>XGA · 1024×768</option>"
            "<option value='UXGA' data-dim='UXGA 1600×1200'>UXGA · 1600×1200</option>"
            "</select></div>"
            "<div class='ctl'><div class='row'><label for='quality'>JPEG quality</label><span class='val' id='qualityV'>12</span></div><input id='quality' type='range' min='4' max='30' value='12'></div>"
            "<div class='ctl'><div class='row'><label for='brightness'>Brightness</label><span class='val' id='brightnessV'>0</span></div><input id='brightness' type='range' min='-2' max='2' value='0'></div>"
            "<div class='ctl'><div class='row'><label for='contrast'>Contrast</label><span class='val' id='contrastV'>0</span></div><input id='contrast' type='range' min='-2' max='2' value='0'></div>"
            "<div class='ctl'><div class='row'><label for='saturation'>Saturation</label><span class='val' id='saturationV'>0</span></div><input id='saturation' type='range' min='-2' max='2' value='0'></div>"
            "<div class='ctl switch'><label for='hmirror'>Mirror</label><label class='tgl'><input id='hmirror' type='checkbox'><span class='sl'></span></label></div>"
            "<div class='ctl switch'><label for='vflip'>Flip</label><label class='tgl'><input id='vflip' type='checkbox'><span class='sl'></span></label></div>"
            "</div></section>");
    } else {
        httpd_resp_sendstr_chunk(req,
            "<div class='frame'><div class='offline'><h2>Camera is not initialized</h2>"
            "<p>Check the camera module and reboot the board.</p></div></div>");
    }

    if (mic_ready) {
        httpd_resp_sendstr_chunk(req,
            "<section class='card'><h3>Microphone</h3>"
            "<div class='toolbar' style='margin:0 0 12px'>"
            "<button class='btn primary' id='rec' type='button'>Record 5s</button>"
            "<a class='btn' id='dl' href='/audio.wav?secs=5' download='xiao-audio.wav'>Download WAV</a>"
            "</div>"
            "<audio id='player' controls style='width:100%'></audio>"
            "<div class='meta'><span id='micState'>PDM mono · 16 kHz</span><span>Wake word: coming soon</span></div>"
            "</section>");
    }

    httpd_resp_sendstr_chunk(req, "<section class='card'><h3>Storage &amp; Power</h3><div class='toolbar' style='margin:0'>");
    if (sd_ready) {
        httpd_resp_sendstr_chunk(req, "<button class='btn danger' id='formatSd' type='button'>Format SD</button>");
    }
    httpd_resp_sendstr_chunk(req, "<button class='btn danger' id='reboot' type='button'>Reboot</button></div><div class='meta'><span>");
    httpd_resp_sendstr_chunk(req, sd_status_html);
    httpd_resp_sendstr_chunk(req, "</span><span>");
    httpd_resp_sendstr_chunk(req, sd_ready ? "Storage ready" : "Storage offline");
    httpd_resp_sendstr_chunk(req, "</span></div></section>");

    /* Firmware / OTA card */
    /* Build metadata is compiler/CMake/git controlled ASCII — no HTML escaping
       needed, and large escape buffers here overflow the httpd task stack. */
    const esp_app_desc_t *app = esp_app_get_description();
    char fw_line[128], tools_line[160];
    snprintf(fw_line, sizeof(fw_line), "%s %s (built %s %s)",
             app->project_name, app->version, app->date, app->time);
    snprintf(tools_line, sizeof(tools_line), "ESP-IDF %s / GCC %s", app->idf_ver, __VERSION__);
    httpd_resp_sendstr_chunk(req,
        "<section class='card'><h3>Firmware</h3>"
        "<div class='meta' style='margin:0 0 4px'><span>Running: ");
    httpd_resp_sendstr_chunk(req, fw_line);
    httpd_resp_sendstr_chunk(req, "</span></div><div class='meta' style='margin:0 0 12px'><span>Built with: ");
    httpd_resp_sendstr_chunk(req, tools_line);
    httpd_resp_sendstr_chunk(req,
        "</span></div>"
        "<div class='toolbar' style='margin:0 0 10px'>"
        "<input id='fw' type='file' accept='.bin' style='flex:2;min-width:180px;height:44px;padding:8px;border:1px solid var(--line);border-radius:10px;background:var(--panel2);color:var(--text)'>"
        "<button class='btn primary' id='flash' type='button'>Flash over WiFi</button>"
        "</div>"
        "<div style='height:8px;border-radius:999px;background:var(--line);overflow:hidden'><div id='otabar' style='height:100%;width:0;background:var(--accent);transition:width .2s'></div></div>"
        "<div class='meta'><span id='otaState'>Select a .bin and flash. The board reboots into it automatically.</span></div>"
        "</section>"
        "<section class='card'><h3>Recording (loop to SD)</h3>"
        "<div class='toolbar' style='margin:0 0 10px'>"
        "<button class='btn primary' id='recBtn' type='button'>Start recording</button>"
        "<label class='btn' style='cursor:pointer;gap:10px'>Audio<input id='recAudio' type='checkbox' style='width:18px;height:18px'></label>"
        "</div>"
        "<div class='ctl'><div class='row'><label for='recPct'>Disk budget</label><span class='val' id='recPctV'>80%</span></div><input id='recPct' type='range' min='10' max='95' value='80'></div>"
        "<div style='height:8px;border-radius:999px;background:var(--line);overflow:hidden;margin:10px 0'><div id='recBar' style='height:100%;width:0;background:var(--accent);transition:width .3s'></div></div>"
        "<div class='meta'><span id='recState'>idle</span><span id='recUse'></span></div>"
        "<div id='clips' style='display:flex;flex-direction:column;gap:6px;margin-top:8px'></div>"
        "</section>"
        "<section class='card'><h3>SD Files</h3>"
        "<div class='toolbar' style='margin:0 0 10px'><button class='btn' id='cfgBackup' type='button'>Backup config to SD</button></div>"
        "<div class='meta' style='margin:0 0 10px'><span id='cfgState'>Config (incl. WiFi) can be saved to SD and auto-loaded on a fresh board.</span></div>"
        "<div class='meta' style='margin:0 0 10px'><span id='fbPath'>/sdcard</span><button class='btn' id='fbUp' type='button' style='flex:0;min-width:70px;height:32px'>Up</button></div>"
        "<div id='fbList' style='display:flex;flex-direction:column;gap:6px'></div>"
        "</section>"
        "</main><script>");

    if (camera_ready) {
        httpd_resp_sendstr_chunk(req,
            "const stream='http://'+location.hostname+':81/stream';"
            "const cam=document.getElementById('cam'),raw=document.getElementById('raw'),res=document.getElementById('res');"
            "cam.src=stream;raw.href=stream;"
            "cam.onerror=()=>{setTimeout(()=>{cam.src=stream+'?t='+Date.now();},1500);};"
            "const ids=['framesize','quality','brightness','contrast','saturation','hmirror','vflip'];"
            "async function setOne(id){const e=document.getElementById(id);const v=e.type==='checkbox'?(e.checked?1:0):e.value;try{await fetch('/control?'+id+'='+encodeURIComponent(v));}catch(_){}}"
            "ids.forEach(id=>{const e=document.getElementById(id);e&&e.addEventListener('change',()=>setOne(id));});"
            "['quality','brightness','contrast','saturation'].forEach(id=>{const e=document.getElementById(id),v=document.getElementById(id+'V');e.addEventListener('input',()=>{v.textContent=e.value;});});"
            "const fs=document.getElementById('framesize');fs.addEventListener('change',()=>{res.textContent=fs.selectedOptions[0].dataset.dim;});"
            "document.getElementById('snap').onclick=()=>{try{const c=document.createElement('canvas');c.width=cam.naturalWidth||640;c.height=cam.naturalHeight||480;c.getContext('2d').drawImage(cam,0,0,c.width,c.height);const a=document.createElement('a');a.download='xiao-snapshot.png';a.href=c.toDataURL('image/png');a.click();}catch(e){alert('Snapshot failed: '+e.message);}};"
            "document.getElementById('full').onclick=()=>{const f=document.querySelector('.frame');const rq=f.requestFullscreen||f.webkitRequestFullscreen;if(rq)rq.call(f);};");
    }
    httpd_resp_sendstr_chunk(req,
        "const _f=document.getElementById('formatSd');if(_f)_f.onclick=async()=>{if(!confirm('Format SD card? This erases the card.'))return;const r=await fetch('/format_sd',{method:'POST'});alert(await r.text());location.reload();};"
        "const _r=document.getElementById('reboot');if(_r)_r.onclick=()=>{if(confirm('Reboot board?'))fetch('/reboot');};"
        "const _rec=document.getElementById('rec');if(_rec)_rec.onclick=async()=>{const st=document.getElementById('micState'),pl=document.getElementById('player');_rec.disabled=true;const t0=st.textContent;st.textContent='Recording 5s...';try{const r=await fetch('/audio.wav?secs=5');const b=await r.blob();pl.src=URL.createObjectURL(b);pl.play().catch(()=>{});st.textContent='Captured '+(b.size>>10)+' KB';}catch(e){st.textContent='Record failed';}_rec.disabled=false;setTimeout(()=>{st.textContent=t0;},4000);};"
        "const _fl=document.getElementById('flash');if(_fl)_fl.onclick=()=>{const f=document.getElementById('fw').files[0],st=document.getElementById('otaState'),bar=document.getElementById('otabar');if(!f){st.textContent='Pick a .bin first';return;}if(!confirm('Flash '+f.name+' ('+(f.size>>10)+' KB) over WiFi?'))return;_fl.disabled=true;const x=new XMLHttpRequest();x.open('POST','/ota');x.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);bar.style.width=p+'%';st.textContent='Uploading '+p+'%';}};x.onload=()=>{st.textContent=x.status===200?'Flashed. Rebooting... reload in ~10s.':'OTA failed: '+x.responseText;if(x.status===200){bar.style.width='100%';setTimeout(()=>location.reload(),12000);}else _fl.disabled=false;};x.onerror=()=>{st.textContent='Upload error';_fl.disabled=false;};x.send(f);};"
        /* Recording controls */
        "const recBtn=document.getElementById('recBtn'),recPct=document.getElementById('recPct');"
        "async function recRefresh(){try{const d=await (await fetch('/rec/list')).json();"
        "recBtn.textContent=d.enabled?'Stop recording':'Start recording';recBtn.classList.toggle('danger',d.enabled);"
        "document.getElementById('recState').textContent=d.status;document.getElementById('recAudio').checked=d.audio;"
        "const budget=d.total*d.pct/100;document.getElementById('recBar').style.width=Math.min(100,budget?d.used/budget*100:0)+'%';"
        "document.getElementById('recUse').textContent=((d.used/1048576)|0)+' MB / '+((budget/1048576)|0)+' MB';"
        "if(document.activeElement!==recPct){recPct.value=d.pct;document.getElementById('recPctV').textContent=d.pct+'%';}"
        "const c=document.getElementById('clips');c.innerHTML='';d.clips.sort((a,b)=>b.name.localeCompare(a.name)).forEach(cl=>{const r=document.createElement('div');r.style.cssText='display:flex;justify-content:space-between;align-items:center;gap:8px;color:var(--muted);font-size:13px';r.innerHTML=\"<span>\"+cl.name+' ('+((cl.size/1048576)|0)+' MB)</span>';const a=document.createElement('a');a.className='btn';a.style.cssText='flex:0;min-width:90px;height:32px';a.textContent='Download';a.href='/download?path=/sdcard/rec/'+encodeURIComponent(cl.name);r.appendChild(a);c.appendChild(r);});}catch(e){}}"
        "recBtn.onclick=async()=>{await fetch('/rec?on='+(recBtn.textContent.startsWith('Start')?1:0),{method:'POST'});setTimeout(recRefresh,300);};"
        "document.getElementById('recAudio').onchange=async e=>{await fetch('/rec?audio='+(e.target.checked?1:0),{method:'POST'});};"
        "recPct.addEventListener('input',()=>document.getElementById('recPctV').textContent=recPct.value+'%');"
        "recPct.addEventListener('change',()=>fetch('/rec?pct='+recPct.value,{method:'POST'}));"
        "setInterval(recRefresh,3000);recRefresh();"
        /* SD file browser */
        "let fbCur='/sdcard';"
        "async function fbLoad(p){try{const r=await fetch('/files?dir='+encodeURIComponent(p));if(!r.ok)return;const d=await r.json();fbCur=d.dir;document.getElementById('fbPath').textContent=d.dir;const l=document.getElementById('fbList');l.innerHTML='';d.entries.sort((a,b)=>(b.dir-a.dir)||a.name.localeCompare(b.name)).forEach(en=>{const row=document.createElement('div');row.style.cssText='display:flex;justify-content:space-between;align-items:center;gap:8px;color:var(--muted);font-size:13px';const s=document.createElement('span');s.textContent=(en.dir?'[dir] ':'')+en.name+(en.dir?'':' ('+((en.size/1024)|0)+' KB)');if(en.dir){s.style.cursor='pointer';s.onclick=()=>fbLoad(d.dir+'/'+en.name);}row.appendChild(s);if(!en.dir){const w=document.createElement('span');const g=document.createElement('a');g.className='btn';g.style.cssText='flex:0;min-width:70px;height:30px';g.textContent='Get';g.href='/download?path='+encodeURIComponent(d.dir+'/'+en.name);const del=document.createElement('button');del.className='btn danger';del.type='button';del.style.cssText='flex:0;min-width:64px;height:30px;margin-left:6px';del.textContent='Del';del.onclick=async()=>{if(confirm('Delete '+en.name+'?')){await fetch('/delete?path='+encodeURIComponent(d.dir+'/'+en.name),{method:'POST'});fbLoad(fbCur);}};w.appendChild(g);w.appendChild(del);row.appendChild(w);}l.appendChild(row);});}catch(e){}}"
        "document.getElementById('fbUp').onclick=()=>{if(fbCur!=='/sdcard')fbLoad(fbCur.substring(0,fbCur.lastIndexOf('/'))||'/sdcard');};"
        "document.getElementById('cfgBackup').onclick=async()=>{const st=document.getElementById('cfgState');st.textContent='Saving...';try{const r=await fetch('/config/backup',{method:'POST'});st.textContent=await r.text();fbLoad(fbCur);}catch(e){st.textContent='Backup failed';}};"
        "fbLoad('/sdcard');"
        "</script></body></html>");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static httpd_uri_t uri_index = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_stream = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_control = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = control_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_reboot = {
    .uri = "/reboot",
    .method = HTTP_GET,
    .handler = reboot_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_format_sd = {
    .uri = "/format_sd",
    .method = HTTP_POST,
    .handler = format_sd_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_audio = {
    .uri = "/audio.wav",
    .method = HTTP_GET,
    .handler = audio_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_ota = {
    .uri = "/ota",
    .method = HTTP_POST,
    .handler = ota_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_rec        = { .uri = "/rec",       .method = HTTP_POST, .handler = rec_toggle_handler, .user_ctx = NULL };
static httpd_uri_t uri_rec_list   = { .uri = "/rec/list",  .method = HTTP_GET,  .handler = rec_list_handler,   .user_ctx = NULL };
static httpd_uri_t uri_files      = { .uri = "/files",     .method = HTTP_GET,  .handler = files_list_handler, .user_ctx = NULL };
static httpd_uri_t uri_download   = { .uri = "/download",  .method = HTTP_GET,  .handler = download_handler,   .user_ctx = NULL };
static httpd_uri_t uri_delete     = { .uri = "/delete",    .method = HTTP_POST, .handler = delete_handler,     .user_ctx = NULL };
static httpd_uri_t uri_cfg_backup = { .uri = "/config/backup", .method = HTTP_POST, .handler = config_backup_handler, .user_ctx = NULL };

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.max_resp_headers = 8;
    /* index_handler builds sizable HTML on-stack; 4 KB default is too tight. */
    config.stack_size = 8192;
    /* Short-lived requests; keep socket use small so the LWIP pool is shared
       with the stream server (port 81) and mDNS without exhaustion. */
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    /* Bound blocked I/O so a stalled client on weak WiFi can't wedge the worker. */
    config.recv_wait_timeout = 8;
    config.send_wait_timeout = 8;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_index);
        httpd_register_uri_handler(server, &uri_control);
        httpd_register_uri_handler(server, &uri_reboot);
        httpd_register_uri_handler(server, &uri_format_sd);
        httpd_register_uri_handler(server, &uri_audio);
        httpd_register_uri_handler(server, &uri_ota);
        httpd_register_uri_handler(server, &uri_rec);
        httpd_register_uri_handler(server, &uri_rec_list);
        httpd_register_uri_handler(server, &uri_files);
        httpd_register_uri_handler(server, &uri_download);
        httpd_register_uri_handler(server, &uri_delete);
        httpd_register_uri_handler(server, &uri_cfg_backup);
        ESP_LOGI(TAG, "HTTP server started");
    }
    return server;
}

static httpd_handle_t start_stream_webserver(void) {
    /* Worker pool for concurrent MJPEG clients (async request handlers). */
    stream_slots = xSemaphoreCreateCounting(STREAM_MAX_CLIENTS, STREAM_MAX_CLIENTS);
    stream_queue = xQueueCreate(STREAM_MAX_CLIENTS, sizeof(httpd_req_t *));
    if (!stream_slots || !stream_queue) {
        ESP_LOGE(TAG, "Stream worker pool alloc failed");
        return NULL;
    }
    for (int i = 0; i < STREAM_MAX_CLIENTS; i++) {
        xTaskCreate(stream_worker, "stream_wrk", 4096, NULL, 5, NULL);
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32769;
    config.max_uri_handlers = 4;
    config.max_resp_headers = 8;
    /* One socket per concurrent stream, plus headroom to accept + reject extras. */
    config.max_open_sockets = STREAM_MAX_CLIENTS + 2;
    config.lru_purge_enable = false;
    config.recv_wait_timeout = 8;
    config.send_wait_timeout = 8;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        ESP_LOGI(TAG, "Stream server started on port 81 (max %d viewers)", STREAM_MAX_CLIENTS);
    }
    return server;
}

/* Provisioning Web Server Handlers */
static esp_err_t prov_index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>XIAO Setup</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        ":root{color-scheme:light;--bg:#eef2f5;--card:#fff;--text:#0f1b24;--muted:#5f7280;--line:#dbe3e8;--accent:#0ea394;--accent-d:#0b8578;}"
        "*{box-sizing:border-box}body{margin:0;color:var(--text);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;-webkit-font-smoothing:antialiased;background:linear-gradient(180deg,#e7f0f2,#eef2f5) fixed;}"
        "main{min-height:100vh;display:grid;place-items:center;padding:18px}.panel{width:min(100%,440px);background:var(--card);border:1px solid var(--line);border-radius:16px;padding:26px;box-shadow:0 20px 50px rgba(20,40,55,.14);}"
        ".logo{width:46px;height:46px;border-radius:12px;background:linear-gradient(135deg,var(--accent),#0b6e8a);display:grid;place-items:center;color:#fff;font-size:22px;margin-bottom:14px}"
        "h1{font-size:22px;line-height:1.2;margin:0 0 6px}.sub{margin:0 0 18px;color:var(--muted);font-size:14px;line-height:1.5}"
        "label{display:block;margin:16px 0 7px;font-weight:600;font-size:13px;color:var(--muted)}"
        "select,input{width:100%;height:46px;border:1px solid var(--line);border-radius:10px;padding:0 12px;font-size:16px;background:#fff;color:var(--text)}"
        "select:focus,input:focus{outline:none;border-color:var(--accent);box-shadow:0 0 0 3px rgba(14,163,148,.15)}"
        ".row{display:flex;gap:10px;align-items:center}.row select{flex:1}.scan{width:52px;flex:none;height:46px;border:1px solid var(--line);border-radius:10px;background:#eef5f6;color:var(--accent);font-size:20px;cursor:pointer}.scan:disabled{opacity:.5}"
        ".pw{position:relative}.pw input{padding-right:70px}.eye{position:absolute;right:6px;top:6px;height:34px;width:56px;border:0;border-radius:8px;background:#eef5f6;color:var(--accent);font-size:12px;font-weight:700;cursor:pointer}"
        ".primary{width:100%;height:50px;margin-top:22px;border:0;border-radius:12px;background:linear-gradient(135deg,var(--accent),var(--accent-d));color:#fff;font-size:16px;font-weight:700;cursor:pointer}.primary:disabled{background:#aeb9bf;cursor:not-allowed}"
        ".hint{margin-top:14px;color:var(--muted);font-size:13px;line-height:1.5}.err{display:none;margin-top:12px;padding:10px 12px;border-radius:10px;background:#fdecea;color:#a12820;font-size:13px}"
        "@keyframes spin{to{transform:rotate(360deg)}}.spin{display:inline-block;animation:spin .8s linear infinite}"
        "</style></head><body><main><section class='panel'>"
        "<div class='logo'>&#9673;</div>"
        "<h1>Connect camera to WiFi</h1><p class='sub'>Pick a network, enter its password, and the board reboots into camera mode.</p>"
        "<form method='POST' action='/save'><label for='ssid'>WiFi network</label><div class='row'><select name='ssid' id='ssid' required>");

    if (scan_result_count == 0) {
        httpd_resp_sendstr_chunk(req, "<option value=''>No networks found</option>");
    } else {
        httpd_resp_sendstr_chunk(req, "<option value=''>Select network...</option>");
        for (int i = 0; i < scan_result_count; i++) {
            char ssid_html[sizeof(scan_results[i].ssid) * 6 + 1];
            char option[sizeof(ssid_html) * 2 + 96];
            html_escape_string(ssid_html, sizeof(ssid_html), (const char *)scan_results[i].ssid);
            snprintf(option, sizeof(option), "<option value='%s'>%s (%d dBm)</option>",
                     ssid_html, ssid_html, scan_results[i].rssi);
            httpd_resp_sendstr_chunk(req, option);
        }
    }

    httpd_resp_sendstr_chunk(req,
        "</select><button class='scan' type='button' id='scanBtn' aria-label='Rescan networks'>&#10227;</button></div>"
        "<label for='pass'>Password</label><div class='pw'><input name='pass' id='pass' type='password' autocomplete='current-password' placeholder='Leave empty for open network'><button type='button' class='eye' id='eye'>Show</button></div>"
        "<button class='primary' id='saveBtn' type='submit' disabled>Save &amp; Reboot</button>"
        "<div id='err' class='err'></div><p class='hint'>Setup AP: <b>XIAO-SETUP</b>. After saving, reconnect your phone or laptop to the selected WiFi and open the IP printed in the serial monitor.</p></form>"
        "<script>"
        "const sel=document.getElementById('ssid'),save=document.getElementById('saveBtn'),err=document.getElementById('err'),btn=document.getElementById('scanBtn');"
        "function sync(){save.disabled=!sel.value}sel.addEventListener('change',sync);sync();"
        "document.getElementById('eye').onclick=function(){const p=document.getElementById('pass');const s=p.type==='password';p.type=s?'text':'password';this.textContent=s?'Hide':'Show';};"
        "function bars(r){const l=r>=-55?4:r>=-67?3:r>=-77?2:1;return '▮'.repeat(l)+'▯'.repeat(4-l);}"
        "btn.onclick=async()=>{btn.disabled=true;btn.innerHTML='<span class=spin>&#10227;</span>';err.style.display='none';try{const r=await fetch('/scan');const nets=await r.json();sel.innerHTML='<option value=\"\">Select network...</option>';nets.forEach(n=>{const o=document.createElement('option');o.value=n.ssid;o.textContent=bars(n.rssi)+'  '+n.ssid+'  ('+n.rssi+' dBm)';sel.appendChild(o)});if(!nets.length){sel.innerHTML='<option value=\"\">No networks found</option>'}sync();}catch(e){err.textContent='Scan failed. Try again.';err.style.display='block'}btn.disabled=false;btn.innerHTML='&#10227;'};"
        "</script></section></main></body></html>");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t scan_wifi_networks(void) {
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {.active = {.min = 100, .max = 300}},
    };
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: 0x%x", err);
        return err;
    }

    uint16_t ap_count = 0;
    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read scan result count: 0x%x", err);
        return err;
    }

    memset(scan_results, 0, sizeof(scan_results));
    scan_result_count = 0;
    if (ap_count == 0) {
        ESP_LOGW(TAG, "WiFi scan found no networks");
        return ESP_OK;
    }

    wifi_ap_record_t *ap_records = calloc(ap_count, sizeof(wifi_ap_record_t));
    if (!ap_records) {
        return ESP_ERR_NO_MEM;
    }

    err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (err != ESP_OK) {
        free(ap_records);
        ESP_LOGE(TAG, "Failed to read scan results: 0x%x", err);
        return err;
    }

    for (int i = 0; i < ap_count && scan_result_count < MAX_SCAN_RESULTS; i++) {
        if (ap_records[i].ssid[0] == '\0') {
            continue;
        }

        bool duplicate = false;
        for (int j = 0; j < scan_result_count; j++) {
            if (strncmp((const char *)scan_results[j].ssid,
                        (const char *)ap_records[i].ssid,
                        sizeof(scan_results[j].ssid)) == 0) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            scan_results[scan_result_count++] = ap_records[i];
        }
    }
    free(ap_records);
    ESP_LOGI(TAG, "WiFi scan cached %u networks", scan_result_count);
    return ESP_OK;
}

static esp_err_t prov_scan_handler(httpd_req_t *req) {
    esp_err_t err = scan_wifi_networks();
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "WiFi scan failed");
        return err;
    }

    httpd_resp_set_type(req, "application/json");
    if (scan_result_count == 0) {
        return httpd_resp_send(req, "[]", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t res = httpd_resp_send_chunk(req, "[", 1);
    for (int i = 0; i < scan_result_count && res == ESP_OK; i++) {
        char escaped_ssid[sizeof(scan_results[i].ssid) * 6 + 1];
        char item[sizeof(escaped_ssid) + 40];
        json_escape_string(escaped_ssid, sizeof(escaped_ssid), (const char *)scan_results[i].ssid);
        int len = snprintf(item, sizeof(item), "%s{\"ssid\":\"%s\",\"rssi\":%d}",
                           i > 0 ? "," : "", escaped_ssid, scan_results[i].rssi);
        if (len < 0 || len >= sizeof(item)) {
            res = ESP_FAIL;
            break;
        }
        res = httpd_resp_send_chunk(req, item, len);
    }
    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, "]", 1);
    }
    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, NULL, 0);
    }
    return res;
}

static esp_err_t prov_save_handler(httpd_req_t *req) {
    char buf[256];
    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty request");
        return ESP_FAIL;
    }
    if (req->content_len >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Request too large");
        return ESP_FAIL;
    }

    size_t received = 0;
    while (received < req->content_len) {
        int len = httpd_req_recv(req, buf + received, req->content_len - received);
        if (len <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request");
            return ESP_FAIL;
        }
        received += len;
    }
    buf[received] = '\0';

    char ssid[32] = {0}, pass[64] = {0};
    form_get_value(buf, "ssid", ssid, sizeof(ssid));
    form_get_value(buf, "pass", pass, sizeof(pass));

    if (strlen(ssid) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }
    if (!ssid_is_scanned(ssid)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID was not found in scan results");
        return ESP_FAIL;
    }
    if (!wifi_password_is_valid(pass)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Password must be empty or 8-63 printable ASCII characters");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Saving WiFi: SSID=%s", ssid);
    esp_err_t err = wifi_credentials_save(ssid, pass);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS save failed");
        return ESP_FAIL;
    }
    sd_config_save(ssid, pass);   /* mirror to SD so the config is portable */

    const char *resp = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                       "<title>Saved</title></head><body style='font-family:sans-serif;"
                       "text-align:center;padding:2rem;'><h2>Saved</h2>"
                       "<p>Rebooting to connect to <b>%s</b>...</p>"
                       "<p><small>If this page doesn't redirect, reconnect to your WiFi.</small></p>"
                       "<meta http-equiv='refresh' content='5; url=/'>"
                       "</body></html>";
    char resp_buf[512];
    snprintf(resp_buf, sizeof(resp_buf), resp, ssid);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_buf, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static esp_err_t prov_save_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_sendstr(req, "Return to setup");
}

static httpd_handle_t start_prov_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start provisioning web server");
        return NULL;
    }

    httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = prov_index_handler };
    httpd_uri_t uri_scan = { .uri = "/scan", .method = HTTP_GET, .handler = prov_scan_handler };
    httpd_uri_t uri_save_get = { .uri = "/save", .method = HTTP_GET, .handler = prov_save_get_handler };
    httpd_uri_t uri_save = { .uri = "/save", .method = HTTP_POST, .handler = prov_save_handler };

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_scan);
    httpd_register_uri_handler(server, &uri_save_get);
    httpd_register_uri_handler(server, &uri_save);

    ESP_LOGI(TAG, "Provisioning web server started on port 80");
    return server;
}

static void json_escape_string(char *dst, size_t dst_size, const char *src) {
    size_t j = 0;
    for (size_t i = 0; src[i] && j + 1 < dst_size; i++) {
        unsigned char c = (unsigned char)src[i];
        const char *escape = NULL;
        char unicode_escape[7];

        switch (c) {
        case '"': escape = "\\\""; break;
        case '\\': escape = "\\\\"; break;
        case '\b': escape = "\\b"; break;
        case '\f': escape = "\\f"; break;
        case '\n': escape = "\\n"; break;
        case '\r': escape = "\\r"; break;
        case '\t': escape = "\\t"; break;
        default:
            if (c < 0x20) {
                snprintf(unicode_escape, sizeof(unicode_escape), "\\u%04x", c);
                escape = unicode_escape;
            }
            break;
        }

        if (escape) {
            size_t escape_len = strlen(escape);
            if (j + escape_len >= dst_size) break;
            memcpy(&dst[j], escape, escape_len);
            j += escape_len;
        } else {
            dst[j++] = (char)c;
        }
    }
    dst[j] = '\0';
}

static void html_escape_string(char *dst, size_t dst_size, const char *src) {
    size_t j = 0;
    for (size_t i = 0; src[i] && j + 1 < dst_size; i++) {
        const char *escape = NULL;
        switch (src[i]) {
        case '&': escape = "&amp;"; break;
        case '<': escape = "&lt;"; break;
        case '>': escape = "&gt;"; break;
        case '"': escape = "&quot;"; break;
        case '\'': escape = "&#39;"; break;
        default: break;
        }

        if (escape) {
            size_t escape_len = strlen(escape);
            if (j + escape_len >= dst_size) break;
            memcpy(&dst[j], escape, escape_len);
            j += escape_len;
        } else {
            dst[j++] = src[i];
        }
    }
    dst[j] = '\0';
}

static bool ssid_is_scanned(const char *ssid) {
    if (scan_result_count == 0) {
        return true;
    }

    for (int i = 0; i < scan_result_count; i++) {
        if (strncmp(ssid, (const char *)scan_results[i].ssid, sizeof(scan_results[i].ssid)) == 0) {
            return true;
        }
    }
    return false;
}

static bool wifi_password_is_valid(const char *pass) {
    size_t len = strlen(pass);
    if (len == 0) {
        return true;
    }
    if (len < 8 || len > 63) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)pass[i];
        if (c < 0x20 || c > 0x7e) {
            return false;
        }
    }
    return true;
}

static bool framesize_from_name(const char *name, framesize_t *framesize) {
    if (strcmp(name, "QVGA") == 0) {
        *framesize = FRAMESIZE_QVGA;
    } else if (strcmp(name, "VGA") == 0) {
        *framesize = FRAMESIZE_VGA;
    } else if (strcmp(name, "SVGA") == 0) {
        *framesize = FRAMESIZE_SVGA;
    } else if (strcmp(name, "XGA") == 0) {
        *framesize = FRAMESIZE_XGA;
    } else if (strcmp(name, "UXGA") == 0) {
        *framesize = FRAMESIZE_UXGA;
    } else {
        return false;
    }
    return true;
}

static bool form_get_value(const char *body, const char *key, char *dst, size_t dst_size) {
    size_t key_len = strlen(key);
    const char *p = body;

    while (p && *p) {
        if ((p == body || p[-1] == '&') && strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
            const char *value = p + key_len + 1;
            const char *end = strchr(value, '&');
            size_t encoded_len = end ? (size_t)(end - value) : strlen(value);
            char encoded[128];

            if (encoded_len >= sizeof(encoded)) {
                encoded_len = sizeof(encoded) - 1;
            }
            memcpy(encoded, value, encoded_len);
            encoded[encoded_len] = '\0';
            url_decode(dst, encoded, dst_size);
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }

    if (dst_size > 0) {
        dst[0] = '\0';
    }
    return false;
}

/* URL decode helper (used by prov_save_handler) */
static void url_decode(char *dst, const char *src, size_t dst_size) {
    size_t i = 0, j = 0;
    while (src[i] && j + 1 < dst_size) {
        if (src[i] == '%' && src[i+1] && src[i+2]) {
            char hex[3] = {src[i+1], src[i+2], 0};
            dst[j++] = (char)strtol(hex, NULL, 16);
            i += 3;
        } else if (src[i] == '+') {
            dst[j++] = ' ';
            i++;
        } else {
            dst[j++] = src[i++];
        }
    }
    dst[j] = '\0';
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting XIAO ESP32S3 Sense Webcam");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
    if (wdt_err == ESP_ERR_INVALID_STATE) {
        wdt_err = esp_task_wdt_reconfigure(&wdt_config);
    }
    if (wdt_err != ESP_OK) {
        ESP_LOGW(TAG, "Task watchdog setup failed: 0x%x", wdt_err);
    }
    ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without camera; WiFi and web UI will still start");
    }
    mic_lock = xSemaphoreCreateMutex();
    if (init_mic() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without microphone");
    }
    init_sd_card();
    rec_cfg_load();
    wifi_init_sta();
    init_mdns();
    start_webserver();
    start_stream_webserver();
    /* Loop recorder runs continuously; it idles until enabled via the web UI. */
    xTaskCreate(rec_task, "rec_task", 6144, NULL, 4, NULL);
}
