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
#include "img_converters.h"
#include "person_detect.h"
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
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include <sys/statvfs.h>
#include "mdns.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include <time.h>
#if CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif
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
static esp_err_t wifi_list_handler(httpd_req_t *req);
static esp_err_t wifi_add_handler(httpd_req_t *req);
static esp_err_t wifi_del_handler(httpd_req_t *req);
static esp_err_t time_handler(httpd_req_t *req);
static esp_err_t ntp_handler(httpd_req_t *req);
static esp_err_t sysinfo_handler(httpd_req_t *req);
static esp_err_t mask_handler(httpd_req_t *req);
static esp_err_t detect_handler(httpd_req_t *req);
static esp_err_t rec_toggle_handler(httpd_req_t *req);
static esp_err_t rec_list_handler(httpd_req_t *req);
static esp_err_t files_list_handler(httpd_req_t *req);
static esp_err_t download_handler(httpd_req_t *req);
static esp_err_t delete_handler(httpd_req_t *req);
static esp_err_t rename_handler(httpd_req_t *req);
static esp_err_t copy_handler(httpd_req_t *req);
static void rec_enforce_budget(void);
static esp_err_t init_sd_card(void);
static esp_err_t mount_sd_card(bool format_if_mount_failed);
static void wifi_init_sta(void);
static void wifi_init_prov(void);
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

/* ============================== BOARD CONFIG ==============================
 * Pin map for the Seeed XIAO ESP32S3 Sense. To adapt to another ESP32-S3
 * camera board, change the camera / microSD / microphone pins below and set
 * the flash & PSRAM size in sdkconfig.defaults. See README "Porting".
 * ========================================================================= */
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
static char device_ip[16] = "";

/* Known-networks list (main + repeaters/guest/etc.) — connect to whichever is up. */
#define WIFI_MAX_CREDS 5
typedef struct { char ssid[33]; char pass[64]; } wifi_cred_t;
static wifi_cred_t wifi_creds[WIFI_MAX_CREDS];
static int wifi_cred_count;
static volatile bool wifi_selecting;   /* true while trying candidates */
static volatile bool wifi_connected;   /* have a live association + IP */
static int wifi_down_cycles;           /* manager ticks with no connection */
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
#define REC_SEG_HARD_BYTES (1024u * 1024 * 1024)  /* safety cap so a clip never runs away */
static volatile bool rec_enabled;
static bool rec_audio = true;              /* mux audio into clips when mic is present */
static int  rec_budget_pct = 80;           /* keep total clips under this % of the card */
static int  rec_seg_min = 15;              /* rotate to a new clip every N minutes (configurable) */
static char rec_cur_file[48] = "";         /* name of the clip currently being written */

/* Motion-gated recording (heuristic: background subtraction on a 1/8 frame). */
static bool motion_enabled = false;        /* record only while there is motion */
static int  motion_sens = 5;               /* % of pixels changed to count as motion (1..30) */
static volatile bool motion_active = false;
static volatile int  motion_last_score = -1;   /* last measured motion %, -1 = not measured yet */
static bool motion_ml = true;                  /* confirm motion with the ML person detector */
static bool ml_ready = false;                  /* person detector loaded ok */
static int  motion_pconf = 25;                 /* person-confidence % to trigger (10..90) */
#define MOTION_PIX_THRESH 18               /* per-pixel luma delta that counts as changed */
/* Ignore-mask over an 8x6 grid ('1' = ignore that cell, e.g. a window/curtain). */
#define MASK_COLS 8
#define MASK_ROWS 6
static char motion_mask[MASK_COLS * MASK_ROWS + 1] = "000000000000000000000000000000000000000000000000";

/* NTP time sync (opt-in; off by default). When synced, clips are named by time. */
#define SYS_NVS_NS "sys_cfg"
static bool ntp_enabled = false;
static char ntp_tz[48] = "UTC0";
static volatile bool time_synced = false;
static bool sntp_started = false;
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

/* ---- Known-networks list (main + repeaters/guest/...) ---- */
static void wifi_creds_save_all(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "n", wifi_cred_count);
    for (int i = 0; i < wifi_cred_count; i++) {
        char ks[12], kp[12];
        snprintf(ks, sizeof(ks), "s%d", i);
        snprintf(kp, sizeof(kp), "p%d", i);
        nvs_set_str(h, ks, wifi_creds[i].ssid);
        nvs_set_str(h, kp, wifi_creds[i].pass);
    }
    nvs_commit(h);
    nvs_close(h);
}

static void wifi_creds_add(const char *ssid, const char *pass) {
    if (!ssid || !ssid[0]) return;
    for (int i = 0; i < wifi_cred_count; i++) {
        if (strcmp(wifi_creds[i].ssid, ssid) == 0) {            /* update existing */
            strlcpy(wifi_creds[i].pass, pass ? pass : "", sizeof(wifi_creds[i].pass));
            wifi_creds_save_all();
            return;
        }
    }
    if (wifi_cred_count >= WIFI_MAX_CREDS) {                    /* full: drop oldest */
        memmove(&wifi_creds[0], &wifi_creds[1], (WIFI_MAX_CREDS - 1) * sizeof(wifi_cred_t));
        wifi_cred_count = WIFI_MAX_CREDS - 1;
    }
    strlcpy(wifi_creds[wifi_cred_count].ssid, ssid, sizeof(wifi_creds[0].ssid));
    strlcpy(wifi_creds[wifi_cred_count].pass, pass ? pass : "", sizeof(wifi_creds[0].pass));
    wifi_cred_count++;
    wifi_creds_save_all();
}

static void wifi_creds_load(void) {
    wifi_cred_count = 0;
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    int32_t n = 0;
    nvs_get_i32(h, "n", &n);
    for (int i = 0; i < n && wifi_cred_count < WIFI_MAX_CREDS; i++) {
        char ks[12], kp[12];
        snprintf(ks, sizeof(ks), "s%d", i);
        snprintf(kp, sizeof(kp), "p%d", i);
        size_t sl = sizeof(wifi_creds[wifi_cred_count].ssid);
        if (nvs_get_str(h, ks, wifi_creds[wifi_cred_count].ssid, &sl) != ESP_OK) continue;
        size_t pl = sizeof(wifi_creds[wifi_cred_count].pass);
        if (nvs_get_str(h, kp, wifi_creds[wifi_cred_count].pass, &pl) != ESP_OK)
            wifi_creds[wifi_cred_count].pass[0] = '\0';
        if (wifi_creds[wifi_cred_count].ssid[0]) wifi_cred_count++;
    }
    if (wifi_cred_count == 0) {   /* migrate a legacy single credential */
        char ssid[33] = {0}, pass[64] = {0};
        size_t sl = sizeof(ssid), pl = sizeof(pass);
        if (nvs_get_str(h, NVS_WIFI_SSID_KEY, ssid, &sl) == ESP_OK && ssid[0]) {
            if (nvs_get_str(h, NVS_WIFI_PASS_KEY, pass, &pl) != ESP_OK) pass[0] = '\0';
            strlcpy(wifi_creds[0].ssid, ssid, sizeof(wifi_creds[0].ssid));
            strlcpy(wifi_creds[0].pass, pass, sizeof(wifi_creds[0].pass));
            wifi_cred_count = 1;
        }
    }
    nvs_close(h);
}

/* Write the full config (all known networks) to the SD card. */
static void sd_config_save_all(void) {
    if (!sd_ready) return;
    FILE *f = fopen(SD_CONFIG_PATH, "w");
    if (!f) {
        ESP_LOGW(TAG, "Could not write %s", SD_CONFIG_PATH);
        return;
    }
    fprintf(f, "# XIAO ESP32S3 camera config\n");
    fprintf(f, "# Repeat ssid/pass for each known network (main, repeaters, ...).\n");
    fprintf(f, "# Drop this SD card into a fresh board to auto-provision.\n");
    for (int i = 0; i < wifi_cred_count; i++) {
        fprintf(f, "ssid=%s\n", wifi_creds[i].ssid);
        fprintf(f, "pass=%s\n", wifi_creds[i].pass);
    }
    fprintf(f, "rec_pct=%d\n", rec_budget_pct);
    fprintf(f, "rec_audio=%d\n", rec_audio ? 1 : 0);
    fprintf(f, "rec_segmin=%d\n", rec_seg_min);
    fprintf(f, "ntp=%d\n", ntp_enabled ? 1 : 0);
    fprintf(f, "tz=%s\n", ntp_tz);
    fprintf(f, "motion=%d\n", motion_enabled ? 1 : 0);
    fprintf(f, "msens=%d\n", motion_sens);
    fprintf(f, "mask=%s\n", motion_mask);
    fclose(f);
    ESP_LOGI(TAG, "Config written to %s (%d networks)", SD_CONFIG_PATH, wifi_cred_count);
}

/* Merge known networks from the SD config into the list. Returns true if any
   credential was added. Also applies recording settings if present. */
static bool sd_config_load(void) {
    FILE *f = fopen(SD_CONFIG_PATH, "r");
    if (!f) return false;
    char line[160], cur_ssid[33] = "";
    bool added = false;
    while (fgets(line, sizeof(line), f)) {
        char *nl = strpbrk(line, "\r\n");
        if (nl) *nl = '\0';
        if (line[0] == '#' || line[0] == '\0') continue;
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        const char *k = line, *v = eq + 1;
        if (strcmp(k, "ssid") == 0) {
            if (cur_ssid[0]) { wifi_creds_add(cur_ssid, ""); added = true; }  /* prev open net */
            strlcpy(cur_ssid, v, sizeof(cur_ssid));
        } else if (strcmp(k, "pass") == 0) {
            if (cur_ssid[0] && wifi_password_is_valid(v)) { wifi_creds_add(cur_ssid, v); added = true; }
            cur_ssid[0] = '\0';
        } else if (strcmp(k, "rec_pct") == 0)   { int p = atoi(v); if (p >= 10 && p <= 95) rec_budget_pct = p; }
        else if (strcmp(k, "rec_audio") == 0)   rec_audio = atoi(v) ? true : false;
        else if (strcmp(k, "rec_segmin") == 0)  { int m = atoi(v); if (m >= 1 && m <= 60) rec_seg_min = m; }
        else if (strcmp(k, "ntp") == 0)         ntp_enabled = atoi(v) ? true : false;
        else if (strcmp(k, "tz") == 0)          strlcpy(ntp_tz, v, sizeof(ntp_tz));
        else if (strcmp(k, "motion") == 0)      motion_enabled = atoi(v) ? true : false;
        else if (strcmp(k, "msens") == 0)       { int s = atoi(v); if (s >= 1 && s <= 30) motion_sens = s; }
        else if (strcmp(k, "mask") == 0)        { if (strlen(v) == MASK_COLS * MASK_ROWS) strlcpy(motion_mask, v, sizeof(motion_mask)); }
    }
    if (cur_ssid[0]) { wifi_creds_add(cur_ssid, ""); added = true; }
    fclose(f);
    return added;
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
    if (nvs_get_i32(h, "on", &v) == ESP_OK) rec_enabled = v ? true : false;  /* auto-resume loop after power-up */
    if (nvs_get_i32(h, "segmin", &v) == ESP_OK && v >= 1 && v <= 60) rec_seg_min = v;
    nvs_close(h);
}
static void rec_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(REC_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "pct", rec_budget_pct);
    nvs_set_i32(h, "audio", rec_audio ? 1 : 0);
    nvs_set_i32(h, "on", rec_enabled ? 1 : 0);
    nvs_set_i32(h, "segmin", rec_seg_min);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- NTP time sync (opt-in) ---- */
static void sys_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(SYS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "ntp", &v) == ESP_OK) ntp_enabled = v ? true : false;
    size_t tl = sizeof(ntp_tz);
    nvs_get_str(h, "tz", ntp_tz, &tl);
    if (nvs_get_i32(h, "motion", &v) == ESP_OK) motion_enabled = v ? true : false;
    if (nvs_get_i32(h, "msens", &v) == ESP_OK && v >= 1 && v <= 30) motion_sens = v;
    if (nvs_get_i32(h, "mlc", &v) == ESP_OK) motion_ml = v ? true : false;
    if (nvs_get_i32(h, "pconf", &v) == ESP_OK && v >= 10 && v <= 90) motion_pconf = v;
    size_t ml = sizeof(motion_mask);
    nvs_get_str(h, "mask", motion_mask, &ml);
    nvs_close(h);
}
static void sys_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(SYS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "ntp", ntp_enabled ? 1 : 0);
    nvs_set_str(h, "tz", ntp_tz);
    nvs_set_i32(h, "motion", motion_enabled ? 1 : 0);
    nvs_set_i32(h, "msens", motion_sens);
    nvs_set_i32(h, "mlc", motion_ml ? 1 : 0);
    nvs_set_i32(h, "pconf", motion_pconf);
    nvs_set_str(h, "mask", motion_mask);
    nvs_commit(h);
    nvs_close(h);
}
static void time_sync_cb(struct timeval *tv) {
    time_synced = true;
    ESP_LOGI(TAG, "System time synced via NTP");
}
/* Start or stop SNTP to match ntp_enabled / ntp_tz. Safe to call repeatedly. */
static void ntp_apply(void) {
    setenv("TZ", ntp_tz, 1);
    tzset();
    if (ntp_enabled && !sntp_started) {
        esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        cfg.sync_cb = time_sync_cb;
        cfg.start = true;
        if (esp_netif_sntp_init(&cfg) == ESP_OK) {
            sntp_started = true;
            ESP_LOGI(TAG, "NTP started (tz=%s)", ntp_tz);
        }
    } else if (!ntp_enabled && sntp_started) {
        esp_netif_sntp_deinit();
        sntp_started = false;
        time_synced = false;
        ESP_LOGI(TAG, "NTP stopped");
    }
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

/* Make the still-open clip valid on disk right now: patch the header sizes to
   the current length and flush to the card. If power is lost after this, the
   file is playable up to here (no idx1, but players scan the movi list). */
static void avi_checkpoint(avi_t *w) {
    if (!w->f) return;
    long end = ftell(w->f);
    patch_u32(w->f, w->p_riff, (uint32_t)(end - 8));
    patch_u32(w->f, w->p_totframes, w->vframes);
    patch_u32(w->f, w->p_vidlen, w->vframes);
    if (w->audio) patch_u32(w->f, w->p_audlen, w->abytes / 2);
    patch_u32(w->f, w->p_movisz, 4 + w->movi_bytes);
    fflush(w->f);
    fsync(fileno(w->f));
}

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
    if (time_synced) {   /* real-time name, e.g. clip-20260702-143005.avi */
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        snprintf(out, n, REC_DIR "/clip-%04d%02d%02d-%02d%02d%02d.avi",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        return;
    }
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
    static time_t mtimes[256];
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
        strlcpy(names[n], e->d_name, sizeof(names[n])); sizes[n] = st.st_size; mtimes[n] = st.st_mtime;
        total += st.st_size; n++;
    }
    closedir(d);
    while (total > budget && n > 0) {
        /* Oldest by modification time — robust across numbered and timestamped names. */
        int oldest = 0;
        for (int i = 1; i < n; i++) if (mtimes[i] < mtimes[oldest]) oldest = i;
        char full[300]; snprintf(full, sizeof(full), REC_DIR "/%s", names[oldest]);
        unlink(full);
        ESP_LOGI(TAG, "rec budget: deleted %s", names[oldest]);
        total -= sizes[oldest];
        n--;
        if (oldest != n) { strlcpy(names[oldest], names[n], sizeof(names[oldest])); sizes[oldest] = sizes[n]; mtimes[oldest] = mtimes[n]; }
    }
}

/* Approx luma of an rgb565 pixel (0..255). */
static inline uint8_t rgb565_luma(uint16_t p) {
    int r = (p >> 11) & 0x1f, g = (p >> 5) & 0x3f, b = p & 0x1f;
    return (uint8_t)((((r << 3) * 77) + ((g << 2) * 150) + ((b << 3) * 29)) >> 8);
}

/* Decode the frame at 1/8 scale and return % of pixels that changed vs a slowly
   adapting background. Buffers live in PSRAM. Returns 0 on the first frame. */
static uint8_t *motion_bg = NULL, *motion_rgb = NULL;
static int motion_bg_w = 0, motion_bg_h = 0;
static int motion_score(camera_fb_t *fb) {
    int w = fb->width / 8, h = fb->height / 8;
    size_t npix = (size_t)w * h;
    if (w <= 0 || h <= 0 || npix > 200 * 150) return -2;
    if (!motion_rgb) { motion_rgb = heap_caps_malloc(200 * 150 * 2, MALLOC_CAP_SPIRAM); if (!motion_rgb) return -2; }
    if (!jpg2rgb565(fb->buf, fb->len, motion_rgb, JPG_SCALE_8X)) return -2;
    uint16_t *px = (uint16_t *)motion_rgb;
    if (!motion_bg || motion_bg_w != w || motion_bg_h != h) {
        if (motion_bg) free(motion_bg);
        motion_bg = heap_caps_malloc(npix, MALLOC_CAP_SPIRAM);
        motion_bg_w = w; motion_bg_h = h;
        if (!motion_bg) return -2;
        for (size_t i = 0; i < npix; i++) motion_bg[i] = rgb565_luma(px[i]);
        return -1;
    }
    int changed = 0, counted = 0;
    for (size_t i = 0; i < npix; i++) {
        uint8_t lum = rgb565_luma(px[i]);
        motion_bg[i] = (uint8_t)((motion_bg[i] * 7 + lum) / 8);   /* slow adapt (even masked) */
        int x = i % w, y = i / w;
        int cell = (y * MASK_ROWS / h) * MASK_COLS + (x * MASK_COLS / w);
        if (motion_mask[cell] == '1') continue;                  /* masked-out region */
        int d = (int)lum - (int)motion_bg[i]; if (d < 0) d = -d;
        if (d > MOTION_PIX_THRESH) changed++;
        counted++;
    }
    return counted > 0 ? (int)(changed * 100 / counted) : 0;
}

static void rec_task(void *arg) {
    avi_t w; bool open = false, have_mic = false; char cur[32] = "";
    int16_t *apcm = NULL;
    const int aframe = MIC_SAMPLE_RATE / REC_TARGET_FPS;
    float dcx = 0, dcy = 0;
    int mcheck = 0, mcool = 0;   /* motion sub-sample counter, cooldown frames */
    while (1) {
        if (!rec_enabled || !sd_ready || !camera_ready) {
            if (open) { avi_end(&w); open = false; rec_cur_file[0] = '\0'; rec_enforce_budget(); }
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

        /* Motion gate: when enabled, only record while there is motion (+ tail). */
        if (motion_enabled) {
            if (++mcheck >= 3) {
                mcheck = 0;
                motion_last_score = motion_score(fb);
                if (motion_last_score >= motion_sens) {
                    /* Heuristic fired. If ML is on, only record when it's a person. */
                    bool trigger = (motion_ml && ml_ready) ? (person_in_frame(fb) != 0) : true;
                    if (trigger) mcool = REC_TARGET_FPS * 5;
                }
            }
            if (mcool > 0) { mcool--; motion_active = true; } else motion_active = false;
            if (!motion_active) {
                if (open) { avi_end(&w); open = false; rec_cur_file[0] = '\0'; rec_enforce_budget(); }
                esp_camera_fb_return(fb);
                snprintf(rec_status, sizeof(rec_status), "armed - waiting for motion");
                vTaskDelay(pdMS_TO_TICKS(1000 / REC_TARGET_FPS));
                continue;
            }
        }

        if (!open) {
            mkdir(REC_DIR, 0777);
            char path[80]; rec_next_path(path, sizeof(path));
            const char *bn = strrchr(path, '/'); strlcpy(cur, bn ? bn + 1 : path, sizeof(cur));
            if (avi_begin(&w, path, fb->width, fb->height, REC_TARGET_FPS, have_mic, MIC_SAMPLE_RATE)) {
                open = true;
                strlcpy(rec_cur_file, cur, sizeof(rec_cur_file));
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
        /* Crash-safety: every ~2 s make the on-disk clip valid + flush to card,
           so a power cut / card pull loses at most ~2 s, not the whole clip. */
        if (w.vframes % (REC_TARGET_FPS * 2) == 0) avi_checkpoint(&w);
        uint32_t seg_frames = (uint32_t)rec_seg_min * 60 * REC_TARGET_FPS;
        if (w.vframes >= seg_frames || w.movi_bytes >= REC_SEG_HARD_BYTES) {
            avi_end(&w); open = false; rec_cur_file[0] = '\0'; rec_enforce_budget();
        }
    }
}

static esp_err_t rec_toggle_handler(httpd_req_t *req) {
    char q[64], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "on", v, sizeof(v)) == ESP_OK) { rec_enabled = atoi(v) ? true : false; rec_cfg_save(); }
        if (httpd_query_key_value(q, "pct", v, sizeof(v)) == ESP_OK) { int p = atoi(v); if (p >= 10 && p <= 95) { rec_budget_pct = p; rec_cfg_save(); } }
        if (httpd_query_key_value(q, "audio", v, sizeof(v)) == ESP_OK) { rec_audio = atoi(v) ? true : false; rec_cfg_save(); }
        if (httpd_query_key_value(q, "seg", v, sizeof(v)) == ESP_OK) { int m = atoi(v); if (m >= 1 && m <= 60) { rec_seg_min = m; rec_cfg_save(); } }
        if (httpd_query_key_value(q, "motion", v, sizeof(v)) == ESP_OK) { motion_enabled = atoi(v) ? true : false; sys_cfg_save(); }
        if (httpd_query_key_value(q, "msens", v, sizeof(v)) == ESP_OK) { int s = atoi(v); if (s >= 1 && s <= 30) { motion_sens = s; sys_cfg_save(); } }
        if (httpd_query_key_value(q, "ml", v, sizeof(v)) == ESP_OK) { motion_ml = atoi(v) ? true : false; sys_cfg_save(); }
        if (httpd_query_key_value(q, "pconf", v, sizeof(v)) == ESP_OK) { int p = atoi(v); if (p >= 10 && p <= 90) { motion_pconf = p; person_set_thr_pct(p); sys_cfg_save(); } }
    }
    char body[200];
    snprintf(body, sizeof(body), "{\"enabled\":%s,\"audio\":%s,\"pct\":%d,\"seg\":%d,\"motion\":%s,\"msens\":%d}",
             rec_enabled ? "true" : "false", rec_audio ? "true" : "false", rec_budget_pct, rec_seg_min,
             motion_enabled ? "true" : "false", motion_sens);
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
            char mt[24]; struct tm tmv; localtime_r(&st.st_mtime, &tmv);
            strftime(mt, sizeof(mt), "%Y-%m-%d %H:%M", &tmv);
            char item[360];
            snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"size\":%lld,\"mtime\":\"%s\"}",
                     first ? "" : ",", e->d_name, (long long)st.st_size, mt);
            first = false;
            httpd_resp_sendstr_chunk(req, item);
        }
        closedir(d);
    }
    char tail[500];
    snprintf(tail, sizeof(tail), "],\"enabled\":%s,\"audio\":%s,\"pct\":%d,\"seg\":%d,\"motion\":%s,\"msens\":%d,\"mactive\":%s,\"mscore\":%d,\"ml\":%s,\"mlready\":%s,\"pconf\":%d,\"mlscore\":%d,\"used\":%llu,\"total\":%llu,\"active\":\"%s\",\"status\":\"%s\"}",
             rec_enabled ? "true" : "false", rec_audio ? "true" : "false", rec_budget_pct, rec_seg_min,
             motion_enabled ? "true" : "false", motion_sens, motion_active ? "true" : "false", motion_last_score,
             motion_ml ? "true" : "false", ml_ready ? "true" : "false", motion_pconf, ml_ready ? person_last_score_pct() : -1,
             (unsigned long long)used, (unsigned long long)total, rec_cur_file, rec_status);
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
        char mt[24]; struct tm tmv; localtime_r(&st.st_mtime, &tmv);
        strftime(mt, sizeof(mt), "%Y-%m-%d %H:%M", &tmv);
        char item[360];
        snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"dir\":%s,\"size\":%lld,\"mtime\":\"%s\"}",
                 first ? "" : ",", esc, S_ISDIR(st.st_mode) ? "true" : "false", (long long)st.st_size, mt);
        first = false;
        httpd_resp_sendstr_chunk(req, item);
    }
    closedir(d);
    httpd_resp_sendstr_chunk(req, "]}");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t do_download(httpd_req_t *req) {
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
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    const char *ctype = strstr(path, ".avi") ? "video/x-msvideo" :
                        strstr(path, ".wav") ? "audio/wav" : "application/octet-stream";
    /* Send the response raw so it carries a real Content-Length (esp_http_server's
       chunked path can't) — that gives the browser a size and progress bar. */
    int sock = httpd_req_to_sockfd(req);
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    /* Never let a stalled client wedge the single port-80 worker forever. */
    struct timeval sto = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &sto, sizeof(sto));
    char hdr[420];
    int hl = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %ld\r\n"
        "Content-Disposition: attachment; filename=\"%s\"\r\n"
        "Accept-Ranges: none\r\nConnection: close\r\n\r\n", ctype, fsz, bn);
    if (httpd_send(req, hdr, hl) < 0) { fclose(f); return ESP_FAIL; }

    const size_t DLBUF = 32768;
    char *buf = malloc(DLBUF);
    if (!buf) { fclose(f); return ESP_FAIL; }
    long remaining = fsz;
    esp_err_t res = ESP_OK;
    while (remaining > 0) {
        size_t want = remaining > (long)DLBUF ? DLBUF : (size_t)remaining;
        size_t r = fread(buf, 1, want, f);
        if (r == 0) break;
        size_t off = 0;
        while (off < r) {
            int s = httpd_send(req, buf + off, r - off);
            if (s <= 0) { res = ESP_FAIL; break; }
            off += s;
        }
        if (res != ESP_OK) break;
        remaining -= r;
    }
    free(buf);
    fclose(f);
    return res;
}

/* Downloads run on their own workers so a big/slow transfer never blocks the
   single port-80 web server (UI, OTA, etc. stay responsive during downloads). */
#define DL_MAX_CONCURRENT 2
static QueueHandle_t dl_queue;
static SemaphoreHandle_t dl_slots;

static void dl_worker(void *arg) {
    httpd_req_t *req;
    while (1) {
        if (xQueueReceive(dl_queue, &req, portMAX_DELAY) == pdTRUE) {
            do_download(req);
            httpd_req_async_handler_complete(req);
            xSemaphoreGive(dl_slots);
        }
    }
}

static esp_err_t download_handler(httpd_req_t *req) {
    if (!dl_slots || xSemaphoreTake(dl_slots, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Too many downloads in progress, try again.");
    }
    httpd_req_t *copy = NULL;
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) {
        xSemaphoreGive(dl_slots);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "download dispatch failed");
        return ESP_FAIL;
    }
    if (xQueueSend(dl_queue, &copy, 0) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        xSemaphoreGive(dl_slots);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "download queue full");
        return ESP_FAIL;
    }
    return ESP_OK;   /* main web worker is free again immediately */
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

/* Parse src+dst query params, both confined to /sdcard. */
static bool sd_two_paths(httpd_req_t *req, char *src, size_t sl, char *dst, size_t dl) {
    char q[420], v[200];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return false;
    src[0] = dst[0] = '\0';
    if (httpd_query_key_value(q, "src", v, sizeof(v)) == ESP_OK) url_decode(src, v, sl);
    if (httpd_query_key_value(q, "dst", v, sizeof(v)) == ESP_OK) url_decode(dst, v, dl);
    return sd_path_ok(src) && sd_path_ok(dst);
}

static esp_err_t rename_handler(httpd_req_t *req) {
    char src[180], dst[180];
    if (!sd_two_paths(req, src, sizeof(src), dst, sizeof(dst))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad src/dst"); return ESP_FAIL;
    }
    if (rename(src, dst) != 0) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "rename failed"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t copy_handler(httpd_req_t *req) {
    char src[180], dst[180];
    if (!sd_two_paths(req, src, sizeof(src), dst, sizeof(dst))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad src/dst"); return ESP_FAIL;
    }
    FILE *in = fopen(src, "rb");
    if (!in) { httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "src not found"); return ESP_FAIL; }
    FILE *out = fopen(dst, "wb");
    if (!out) { fclose(in); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cannot create dst"); return ESP_FAIL; }
    char *buf = malloc(16384);
    bool ok = buf != NULL;
    size_t r;
    while (ok && (r = fread(buf, 1, 16384, in)) > 0) {
        if (fwrite(buf, 1, r, out) != r) ok = false;
    }
    free(buf);
    fclose(in);
    fclose(out);
    if (!ok) { unlink(dst); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "copy failed"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t sysinfo_handler(httpd_req_t *req) {
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    const char *model = chip.model == CHIP_ESP32S3 ? "ESP32-S3" : "ESP32";
    uint32_t flash_sz = 0;
    esp_flash_get_size(NULL, &flash_sz);
    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_tot  = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t ps_free   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t ps_tot    = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    uint64_t up = esp_timer_get_time() / 1000000ULL;
    uint64_t sd_tot = 0, sd_free = 0;
    if (sd_ready) esp_vfs_fat_info(SD_MOUNT_POINT, &sd_tot, &sd_free);
    const esp_app_desc_t *app = esp_app_get_description();

    char body[512];
    snprintf(body, sizeof(body),
        "{\"chip\":\"%s\",\"cores\":%d,\"rev\":%d,\"cpu_mhz\":%d,"
        "\"flash_mb\":%u,\"heap_free\":%u,\"heap_total\":%u,"
        "\"psram_free\":%u,\"psram_total\":%u,"
        "\"sd_free\":%llu,\"sd_total\":%llu,\"uptime\":%llu,\"idf\":\"%s\"}",
        model, chip.cores, chip.revision, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        (unsigned)(flash_sz / (1024 * 1024)), (unsigned)heap_free, (unsigned)heap_tot,
        (unsigned)ps_free, (unsigned)ps_tot,
        (unsigned long long)sd_free, (unsigned long long)sd_tot,
        (unsigned long long)up, app->idf_ver);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

/* Back up the current config (incl. WiFi creds from NVS) to the SD card. */
static esp_err_t config_backup_handler(httpd_req_t *req) {
    if (!sd_ready) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No SD card mounted"); return ESP_FAIL; }
    if (wifi_cred_count == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No saved WiFi to back up");
        return ESP_FAIL;
    }
    sd_config_save_all();
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Config saved to " SD_CONFIG_PATH);
}

static esp_err_t wifi_list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{\"networks\":[");
    for (int i = 0; i < wifi_cred_count; i++) {
        char esc[64], item[80];
        json_escape_string(esc, sizeof(esc), wifi_creds[i].ssid);
        snprintf(item, sizeof(item), "%s\"%s\"", i ? "," : "", esc);
        httpd_resp_sendstr_chunk(req, item);
    }
    wifi_ap_record_t ap;
    int rssi = 0, ch = 0;
    char apssid[33] = "";
    const char *phy = "-";
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        rssi = ap.rssi; ch = ap.primary;
        strlcpy(apssid, (const char *)ap.ssid, sizeof(apssid));
        phy = ap.phy_11n ? "11n" : ap.phy_11g ? "11g" : ap.phy_11b ? "11b" : "-";
    }
    char apesc[64];
    json_escape_string(apesc, sizeof(apesc), apssid);
    char tail[256];
    snprintf(tail, sizeof(tail),
             "],\"count\":%d,\"max\":%d,\"current\":\"%s\",\"ap\":\"%s\",\"rssi\":%d,\"ch\":%d,\"phy\":\"%s\"}",
             wifi_cred_count, WIFI_MAX_CREDS, device_ip[0] ? device_ip : "", apesc, rssi, ch, phy);
    httpd_resp_sendstr_chunk(req, tail);
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t wifi_add_handler(httpd_req_t *req) {
    char q[256], ssid[33] = "", pass[64] = "", v[160];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "ssid", v, sizeof(v)) == ESP_OK) url_decode(ssid, v, sizeof(ssid));
        if (httpd_query_key_value(q, "pass", v, sizeof(v)) == ESP_OK) url_decode(pass, v, sizeof(pass));
    }
    if (!ssid[0] || !wifi_password_is_valid(pass)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required; password empty or 8-63 chars");
        return ESP_FAIL;
    }
    wifi_creds_add(ssid, pass);
    sd_config_save_all();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t wifi_del_handler(httpd_req_t *req) {
    char q[128], ssid[33] = "", v[96];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "ssid", v, sizeof(v)) == ESP_OK) {
        url_decode(ssid, v, sizeof(ssid));
    }
    for (int i = 0; i < wifi_cred_count; i++) {
        if (strcmp(wifi_creds[i].ssid, ssid) == 0) {
            memmove(&wifi_creds[i], &wifi_creds[i + 1], (wifi_cred_count - i - 1) * sizeof(wifi_cred_t));
            wifi_cred_count--;
            wifi_creds_save_all();
            sd_config_save_all();
            break;
        }
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* Debug: run the person detector on one live frame and report result + timing. */
static esp_err_t detect_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    if (!ml_ready || !camera_ready) return httpd_resp_sendstr(req, "{\"ready\":false}");
    char q[32], v[8];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "fmt", v, sizeof(v)) == ESP_OK) person_set_pixfmt(atoi(v));
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "thr", v, sizeof(v)) == ESP_OK) person_set_thr_pct(atoi(v));
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return httpd_resp_sendstr(req, "{\"ready\":true,\"error\":\"no frame\"}");
    int64_t t0 = esp_timer_get_time();
    int person = person_in_frame(fb);
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    esp_camera_fb_return(fb);
    char body[160];
    snprintf(body, sizeof(body), "{\"ready\":true,\"person\":%s,\"score\":%d,\"ms\":%d,\"w\":%d,\"h\":%d}",
             person ? "true" : "false", person_last_score_pct(), ms, fb->width, fb->height);
    return httpd_resp_sendstr(req, body);
}

static esp_err_t mask_handler(httpd_req_t *req) {
    char q[128], v[64];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "grid", v, sizeof(v)) == ESP_OK) {
        if (strlen(v) == MASK_COLS * MASK_ROWS) {
            bool ok = true;
            for (int i = 0; v[i]; i++) if (v[i] != '0' && v[i] != '1') { ok = false; break; }
            if (ok) { strlcpy(motion_mask, v, sizeof(motion_mask)); sys_cfg_save(); }
        }
    }
    char body[96];
    snprintf(body, sizeof(body), "{\"cols\":%d,\"rows\":%d,\"grid\":\"%s\"}", MASK_COLS, MASK_ROWS, motion_mask);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t time_handler(httpd_req_t *req) {
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
    char tz_esc[96], body[256];
    json_escape_string(tz_esc, sizeof(tz_esc), ntp_tz);
    snprintf(body, sizeof(body), "{\"ntp\":%s,\"synced\":%s,\"tz\":\"%s\",\"time\":\"%s\"}",
             ntp_enabled ? "true" : "false", time_synced ? "true" : "false", tz_esc, ts);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t ntp_handler(httpd_req_t *req) {
    char q[160], v[96];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "tz", v, sizeof(v)) == ESP_OK) {
            char dec[48];
            url_decode(dec, v, sizeof(dec));
            strlcpy(ntp_tz, dec, sizeof(ntp_tz));
        }
        if (httpd_query_key_value(q, "on", v, sizeof(v)) == ESP_OK) {
            ntp_enabled = atoi(v) ? true : false;
        }
    }
    sys_cfg_save();
    ntp_apply();
    return time_handler(req);
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
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;   /* 20 MHz (SD isn't the download bottleneck) */

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
        if (!wifi_selecting) esp_wifi_connect();   /* during selection we connect manually */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        if (wifi_selecting) {
            /* Candidate attempt failed; let wifi_connect_best() move to the next. */
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        } else {
            /* Quick reconnect to the current AP for transient drops; if it stays
               down, wifi_manager_task re-scans and roams to another known net. */
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(device_ip, sizeof(device_ip), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        wifi_got_ip_once = true;
        wifi_connected = true;
        wifi_down_cycles = 0;
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

/* Scan, then try known networks present on air (best RSSI first). One attempt
   per candidate; returns true as soon as one gets an IP. */
static bool wifi_connect_best(void) {
    scan_wifi_networks();   /* fills scan_results / scan_result_count */

    int order[WIFI_MAX_CREDS], rssi[WIFI_MAX_CREDS], nc = 0;
    for (int i = 0; i < wifi_cred_count; i++) {
        int best = -128; bool present = false;
        for (int j = 0; j < scan_result_count; j++) {
            if (strcmp(wifi_creds[i].ssid, (const char *)scan_results[j].ssid) == 0) {
                present = true;
                if (scan_results[j].rssi > best) best = scan_results[j].rssi;
            }
        }
        if (present) { order[nc] = i; rssi[nc] = best; nc++; }
    }
    for (int a = 0; a < nc; a++)          /* sort candidates by RSSI desc */
        for (int b = a + 1; b < nc; b++)
            if (rssi[b] > rssi[a]) {
                int t = order[a]; order[a] = order[b]; order[b] = t;
                t = rssi[a]; rssi[a] = rssi[b]; rssi[b] = t;
            }

    int tries = nc > 0 ? nc : wifi_cred_count;   /* if none seen (hidden SSID), still try all */
    for (int k = 0; k < tries; k++) {
        int i = nc > 0 ? order[k] : k;
        wifi_config_t wc = {0};
        strlcpy((char *)wc.sta.ssid, wifi_creds[i].ssid, sizeof(wc.sta.ssid));
        strlcpy((char *)wc.sta.password, wifi_creds[i].pass, sizeof(wc.sta.password));
        wc.sta.threshold.authmode = strlen(wifi_creds[i].pass) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        wc.sta.pmf_cfg.capable = true;
        esp_wifi_set_config(WIFI_IF_STA, &wc);
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        ESP_LOGI(TAG, "Trying WiFi '%s'%s...", wifi_creds[i].ssid,
                 nc > 0 ? " (in range)" : "");
        esp_wifi_connect();
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(12000));
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to '%s'", wifi_creds[i].ssid);
            return true;
        }
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return false;
}

/* Roam: if the current network stays down, re-scan and try all known networks. */
static void wifi_manager_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (wifi_selecting) continue;
        if (wifi_connected) { wifi_down_cycles = 0; continue; }
        if (++wifi_down_cycles >= 2) {   /* ~10s down: the current AP isn't coming back */
            ESP_LOGW(TAG, "Network down; scanning to roam across known networks...");
            wifi_selecting = true;
            esp_wifi_disconnect();
            bool ok = wifi_connect_best();
            wifi_selecting = false;
            wifi_down_cycles = 0;
            if (!ok) ESP_LOGW(TAG, "No known network reachable yet; will retry");
        }
    }
}

static void wifi_init_sta(void) {
    wifi_creds_load();               /* known networks from NVS */
    if (sd_config_load()) {          /* merge any networks + settings from the SD card */
        ESP_LOGI(TAG, "Merged known networks from SD (%s)", SD_CONFIG_PATH);
    }
    if (wifi_cred_count == 0) {
        ESP_LOGW(TAG, "No known WiFi networks in NVS or SD, starting provisioning mode");
        wifi_init_prov();
        return;
    }
    ESP_LOGI(TAG, "%d known WiFi network(s) configured", wifi_cred_count);

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();  // needed for potential AP fallback

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta, NULL));

    wifi_selecting = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Always-on camera on mains power: disable modem sleep. v6.0 defaults to
       WIFI_PS_MIN_MODEM, which adds latency and packet loss for a server role. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    bool connected = wifi_connect_best();
    wifi_selecting = false;
    if (connected) {
        /* Keep the link alive and roam across known networks if it drops. */
        xTaskCreate(wifi_manager_task, "wifi_mgr", 4096, NULL, 4, NULL);
        return;
    }
    ESP_LOGW(TAG, "No known network reachable; starting provisioning AP");
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta));
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
        ".maskgrid{position:absolute;inset:0;display:grid;grid-template-columns:repeat(8,1fr);grid-template-rows:repeat(6,1fr);z-index:3}.mcell{border:1px solid rgba(255,255,255,.14);cursor:pointer}.mcell.on{background:rgba(229,72,77,.5)}.mcell:hover{background:rgba(255,255,255,.12)}"
        ".offline{aspect-ratio:4/3;display:grid;place-content:center;text-align:center;color:var(--muted);padding:24px;gap:6px}.offline h2{margin:0;color:var(--text);font-size:20px}.offline p{margin:0}"
        ".toolbar{display:flex;gap:10px;margin:14px 0;flex-wrap:wrap}"
        ".btn{flex:1;min-width:120px;height:44px;border:1px solid var(--line);border-radius:10px;background:var(--panel2);color:var(--text);font-weight:600;font-size:14px;cursor:pointer;display:inline-flex;align-items:center;justify-content:center;gap:8px;text-decoration:none;transition:.15s}"
        ".btn:hover{border-color:var(--accent);background:#1e2a33}.btn.primary{background:linear-gradient(135deg,var(--accent),var(--accent-d));border:0;color:#052723}.btn.danger:hover{border-color:var(--danger);color:#ff8b8e}"
        ".lnk{background:none;border:0;color:var(--accent);cursor:pointer;font-size:13px;padding:0 5px;text-decoration:none}.lnk.danger{color:var(--danger)}"
        "table.fb{width:100%;border-collapse:collapse;font-size:13px}table.fb th{text-align:left;color:var(--muted);font-weight:600;padding:6px 4px;border-bottom:1px solid var(--line)}table.fb td{padding:6px 4px;border-bottom:1px solid var(--line);vertical-align:middle}"
        ".kv{display:grid;grid-template-columns:1fr 1fr;gap:6px 16px}.kv div{display:flex;justify-content:space-between;gap:8px;color:var(--muted);font-size:13px}.kv b{color:var(--text);font-weight:600;font-variant-numeric:tabular-nums}"
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
            "<img id='cam' crossorigin='anonymous' alt='Camera stream'>"
            "<div class='maskgrid' id='maskgrid' style='display:none'></div></div>"
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
        "<div class='ctl'><div class='row'><label for='recSeg'>Clip length</label><span class='val' id='recSegV'>15 min</span></div><input id='recSeg' type='range' min='1' max='60' value='15'></div>"
        "<div class='ctl switch'><label for='recMotion'>Record on motion only</label><label class='tgl'><input id='recMotion' type='checkbox'><span class='sl'></span></label></div>"
        "<div class='ctl'><div class='row'><label for='recMsens'>Motion threshold</label><span class='val' id='recMsensV'>5%</span></div><input id='recMsens' type='range' min='1' max='30' value='5'></div>"
        "<div class='ctl switch'><label for='recMl'>Confirm with AI (person only)</label><label class='tgl'><input id='recMl' type='checkbox'><span class='sl'></span></label></div>"
        "<div class='ctl'><div class='row'><label for='recPconf'>AI person confidence</label><span class='val' id='recPconfV'>25%</span></div><input id='recPconf' type='range' min='10' max='90' value='25'></div>"
        "<div class='toolbar' style='margin:0'><button class='btn' id='maskBtn' type='button'>Edit ignore zones</button></div>"
        "<div style='height:8px;border-radius:999px;background:var(--line);overflow:hidden;margin:10px 0'><div id='recBar' style='height:100%;width:0;background:var(--accent);transition:width .3s'></div></div>"
        "<div class='meta'><span id='recState'>idle</span><span id='recUse'></span></div>"
        "<div id='clips' style='display:flex;flex-direction:column;gap:6px;margin-top:8px'></div>"
        "</section>"
        "<section class='card'><h3>System</h3><div class='kv' id='sysinfo'></div></section>"
        "<section class='card'><h3>Time</h3>"
        "<div class='meta' style='margin:0 0 10px'><span id='clock' style='font-variant-numeric:tabular-nums;font-size:16px;color:var(--text)'>--:--:--</span><span id='ntpState'>NTP off</span></div>"
        "<div class='toolbar' style='margin:0'>"
        "<label class='btn' style='cursor:pointer;gap:10px'>Enable NTP<input id='ntpOn' type='checkbox' style='width:18px;height:18px'></label>"
        "<select id='tz' style='flex:2;min-width:170px;height:40px;border:1px solid var(--line);border-radius:8px;background:var(--panel2);color:var(--text);padding:0 10px'>"
        "<option value='UTC0'>UTC</option>"
        "<option value='GMT0BST,M3.5.0/1,M10.5.0'>Europe/London, Dublin, Lisbon</option>"
        "<option value='CET-1CEST,M3.5.0,M10.5.0/3'>Europe/Berlin, Paris, Madrid, Rome</option>"
        "<option value='EET-2EEST,M3.5.0/3,M10.5.0/4'>Europe/Kyiv, Athens, Helsinki</option>"
        "<option value='MSK-3'>Europe/Moscow, Istanbul</option>"
        "<option value='&lt;-03&gt;3'>America/Sao Paulo, Buenos Aires</option>"
        "<option value='EST5EDT,M3.2.0,M11.1.0'>America/New York, Toronto</option>"
        "<option value='CST6CDT,M3.2.0,M11.1.0'>America/Chicago, Mexico City</option>"
        "<option value='MST7MDT,M3.2.0,M11.1.0'>America/Denver</option>"
        "<option value='PST8PDT,M3.2.0,M11.1.0'>America/Los Angeles, Vancouver</option>"
        "<option value='&lt;+04&gt;-4'>Asia/Dubai, Tbilisi</option>"
        "<option value='IST-5:30'>Asia/Kolkata, Mumbai</option>"
        "<option value='&lt;+07&gt;-7'>Asia/Bangkok, Jakarta</option>"
        "<option value='CST-8'>Asia/Shanghai, Singapore, Perth</option>"
        "<option value='JST-9'>Asia/Tokyo, Seoul</option>"
        "<option value='AEST-10AEDT,M10.1.0,M4.1.0/3'>Australia/Sydney, Melbourne</option>"
        "<option value='NZST-12NZDT,M9.5.0,M4.1.0/3'>Pacific/Auckland</option>"
        "</select>"
        "</div>"
        "</section>"
        "<section class='card'><h3>WiFi networks</h3>"
        "<div id='wifiList' style='display:flex;flex-direction:column;gap:6px;margin-bottom:10px'></div>"
        "<div class='toolbar' style='margin:0 0 8px'>"
        "<input id='wSsid' placeholder='SSID' style='flex:1;min-width:110px;height:40px;border:1px solid var(--line);border-radius:8px;background:var(--panel2);color:var(--text);padding:0 10px'>"
        "<input id='wPass' type='password' placeholder='Password' style='flex:1;min-width:110px;height:40px;border:1px solid var(--line);border-radius:8px;background:var(--panel2);color:var(--text);padding:0 10px'>"
        "<button class='btn primary' id='wAdd' type='button'>Add</button>"
        "</div>"
        "<div class='meta'><span id='wifiState'>The board connects to the strongest known network on boot.</span></div>"
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
        "document.getElementById('recState').textContent=d.status+(d.motion?'  ·  motion '+(d.mscore>=0?d.mscore+'%':'...'):'');document.getElementById('recAudio').checked=d.audio;"
        "const budget=d.total*d.pct/100;document.getElementById('recBar').style.width=Math.min(100,budget?d.used/budget*100:0)+'%';"
        "document.getElementById('recUse').textContent=((d.used/1048576)|0)+' MB / '+((budget/1048576)|0)+' MB';"
        "if(document.activeElement!==recPct){recPct.value=d.pct;document.getElementById('recPctV').textContent=d.pct+'%';}"
        "var _rs=document.getElementById('recSeg');if(document.activeElement!==_rs){_rs.value=d.seg;document.getElementById('recSegV').textContent=d.seg+' min';}"
        "document.getElementById('recMotion').checked=d.motion;var _ms=document.getElementById('recMsens');if(document.activeElement!==_ms){_ms.value=d.msens;document.getElementById('recMsensV').textContent=d.msens+'%';}"
        "var _ml=document.getElementById('recMl');_ml.checked=d.ml;_ml.disabled=!d.mlready;_ml.parentElement.style.opacity=d.mlready?'1':'.5';"
        "var _pc=document.getElementById('recPconf');if(document.activeElement!==_pc){_pc.value=d.pconf;document.getElementById('recPconfV').textContent=d.pconf+'% (now '+(d.mlscore>=0?d.mlscore+'%':'-')+')';}"
        "const c=document.getElementById('clips');c.innerHTML='';d.clips.sort((a,b)=>b.name.localeCompare(a.name)).forEach(cl=>{const live=cl.name===d.active;const r=document.createElement('div');r.style.cssText='display:flex;justify-content:space-between;align-items:center;gap:8px;color:var(--muted);font-size:13px';r.innerHTML='<span>'+(live?'● ':'')+cl.name+' ('+((cl.size/1048576)|0)+' MB)'+(cl.mtime?'  ·  '+cl.mtime:'')+(live?'  · recording':'')+'</span>';if(!live){const a=document.createElement('a');a.className='btn';a.style.cssText='flex:0;min-width:90px;height:32px';a.textContent='Download';a.href='/download?path=/sdcard/rec/'+encodeURIComponent(cl.name);r.appendChild(a);}c.appendChild(r);});}catch(e){}}"
        "recBtn.onclick=async()=>{await fetch('/rec?on='+(recBtn.textContent.startsWith('Start')?1:0),{method:'POST'});setTimeout(recRefresh,300);};"
        "document.getElementById('recAudio').onchange=async e=>{await fetch('/rec?audio='+(e.target.checked?1:0),{method:'POST'});};"
        "recPct.addEventListener('input',()=>document.getElementById('recPctV').textContent=recPct.value+'%');"
        "recPct.addEventListener('change',()=>fetch('/rec?pct='+recPct.value,{method:'POST'}));"
        "var recSeg=document.getElementById('recSeg');recSeg.addEventListener('input',()=>document.getElementById('recSegV').textContent=recSeg.value+' min');"
        "recSeg.addEventListener('change',()=>fetch('/rec?seg='+recSeg.value,{method:'POST'}));"
        "document.getElementById('recMotion').onchange=e=>{fetch('/rec?motion='+(e.target.checked?1:0),{method:'POST'});setTimeout(recRefresh,300);};"
        "document.getElementById('recMl').onchange=e=>{fetch('/rec?ml='+(e.target.checked?1:0),{method:'POST'});setTimeout(recRefresh,300);};"
        "var recPconf=document.getElementById('recPconf');recPconf.addEventListener('input',()=>document.getElementById('recPconfV').textContent=recPconf.value+'%');recPconf.addEventListener('change',()=>fetch('/rec?pconf='+recPconf.value,{method:'POST'}));"
        "var recMsens=document.getElementById('recMsens');recMsens.addEventListener('input',()=>document.getElementById('recMsensV').textContent=recMsens.value+'%');"
        "recMsens.addEventListener('change',()=>fetch('/rec?msens='+recMsens.value,{method:'POST'}));"
        "var mg=document.getElementById('maskgrid'),mb=document.getElementById('maskBtn');"
        "if(mg&&mb){let mgrid='0'.repeat(48);"
        "function mrender(){mg.innerHTML='';for(let i=0;i<mgrid.length;i++){const c=document.createElement('div');c.className='mcell'+(mgrid[i]==='1'?' on':'');c.onclick=async()=>{mgrid=mgrid.substring(0,i)+(mgrid[i]==='1'?'0':'1')+mgrid.substring(i+1);c.classList.toggle('on');await fetch('/mask?grid='+mgrid,{method:'POST'});};mg.appendChild(c);}}"
        "fetch('/mask').then(r=>r.json()).then(d=>{mgrid=d.grid;mg.style.gridTemplateColumns='repeat('+d.cols+',1fr)';mg.style.gridTemplateRows='repeat('+d.rows+',1fr)';mrender();}).catch(()=>{});"
        "mb.onclick=()=>{const show=mg.style.display==='none';mg.style.display=show?'grid':'none';mb.textContent=show?'Done editing zones':'Edit ignore zones';if(show)window.scrollTo({top:0,behavior:'smooth'});};}"
        "setInterval(recRefresh,3000);recRefresh();"
        /* SD file browser */
        "let fbCur='/sdcard';"
        "function fbSize(n){return n>1048576?(n/1048576).toFixed(1)+' MB':((n/1024)|0)+' KB';}"
        "async function fbLoad(p){try{const r=await fetch('/files?dir='+encodeURIComponent(p));if(!r.ok)return;const d=await r.json();fbCur=d.dir;document.getElementById('fbPath').textContent=d.dir;"
        "let h='<table class=fb><thead><tr><th>Name</th><th>Modified</th><th style=text-align:right>Size</th><th style=text-align:right>Actions</th></tr></thead><tbody>';"
        "d.entries.sort((a,b)=>(b.dir-a.dir)||a.name.localeCompare(b.name)).forEach(en=>{const fp=d.dir+'/'+en.name;const nm=en.dir?('[dir] '+en.name):en.name;let a;"
        "if(en.dir){a='<button class=lnk data-a=cd data-p=\"'+fp+'\">open</button>';}else{a='<a class=lnk href=\"/download?path='+encodeURIComponent(fp)+'\">get</a><button class=lnk data-a=ren data-p=\"'+fp+'\">ren</button><button class=lnk data-a=cp data-p=\"'+fp+'\">copy</button><button class=\"lnk danger\" data-a=del data-p=\"'+fp+'\">del</button>';}"
        "h+='<tr><td'+(en.dir?' class=lnk data-a=cd data-p=\"'+fp+'\" style=cursor:pointer':'')+'>'+nm+'</td><td style=color:var(--muted)>'+(en.mtime||'')+'</td><td style=\"text-align:right;color:var(--muted)\">'+(en.dir?'':fbSize(en.size))+'</td><td style=\"text-align:right;white-space:nowrap\">'+a+'</td></tr>';});"
        "h+='</tbody></table>';const L=document.getElementById('fbList');L.innerHTML=h;L.querySelectorAll('[data-a]').forEach(el=>{el.onclick=()=>fbAct(el.dataset.a,el.dataset.p);});}catch(e){}}"
        "async function fbAct(a,fp){const nm=fp.substring(fp.lastIndexOf('/')+1),dir=fp.substring(0,fp.lastIndexOf('/'));"
        "if(a==='cd'){fbLoad(fp);}"
        "else if(a==='del'){if(confirm('Delete '+nm+'?')){await fetch('/delete?path='+encodeURIComponent(fp),{method:'POST'});fbLoad(fbCur);}}"
        "else if(a==='ren'){const nn=prompt('Rename to:',nm);if(nn&&nn!==nm){await fetch('/rename?src='+encodeURIComponent(fp)+'&dst='+encodeURIComponent(dir+'/'+nn),{method:'POST'});fbLoad(fbCur);}}"
        "else if(a==='cp'){const nn=prompt('Copy to name:','copy-'+nm);if(nn){await fetch('/copy?src='+encodeURIComponent(fp)+'&dst='+encodeURIComponent(dir+'/'+nn),{method:'POST'});fbLoad(fbCur);}}}"
        "document.getElementById('fbUp').onclick=()=>{if(fbCur!=='/sdcard')fbLoad(fbCur.substring(0,fbCur.lastIndexOf('/'))||'/sdcard');};"
        "document.getElementById('cfgBackup').onclick=async()=>{const st=document.getElementById('cfgState');st.textContent='Saving...';try{const r=await fetch('/config/backup',{method:'POST'});st.textContent=await r.text();fbLoad(fbCur);}catch(e){st.textContent='Backup failed';}};"
        "async function wifiRefresh(){try{const d=await(await fetch('/wifi/list')).json();const l=document.getElementById('wifiList');l.innerHTML='';d.networks.forEach(ss=>{const r=document.createElement('div');r.style.cssText='display:flex;justify-content:space-between;align-items:center;gap:8px;color:var(--muted);font-size:13px';const s=document.createElement('span');s.textContent=ss;r.appendChild(s);const del=document.createElement('button');del.className='btn danger';del.type='button';del.style.cssText='flex:0;min-width:60px;height:30px';del.textContent='Del';del.onclick=async()=>{await fetch('/wifi/del?ssid='+encodeURIComponent(ss),{method:'POST'});wifiRefresh();};r.appendChild(del);l.appendChild(r);});document.getElementById('wifiState').textContent=(d.ap?d.ap+' · '+d.rssi+' dBm · ch'+d.ch+' · '+d.phy:'not connected')+' · '+d.count+'/'+d.max+' saved'+(d.current?' · '+d.current:'');}catch(e){}}"
        "document.getElementById('wAdd').onclick=async()=>{const ss=document.getElementById('wSsid').value,pw=document.getElementById('wPass').value,st=document.getElementById('wifiState');if(!ss)return;const r=await fetch('/wifi/add?ssid='+encodeURIComponent(ss)+'&pass='+encodeURIComponent(pw),{method:'POST'});if(r.ok){document.getElementById('wSsid').value='';document.getElementById('wPass').value='';wifiRefresh();}else{st.textContent=await r.text();}};"
        "wifiRefresh();"
        "function fmtB(n){return n>=1073741824?(n/1073741824).toFixed(1)+' GB':n>=1048576?(n/1048576).toFixed(1)+' MB':((n/1024)|0)+' KB';}"
        "async function sysRefresh(){try{const d=await(await fetch('/sysinfo')).json();const up=d.uptime,uh=(up/3600|0),um=((up%3600)/60|0);"
        "const rows=[['Chip',d.chip+' rev'+d.rev+', '+d.cores+' cores @ '+d.cpu_mhz+' MHz'],['Flash',d.flash_mb+' MB, ESP-IDF '+d.idf],['RAM free',fmtB(d.heap_free)+' / '+fmtB(d.heap_total)],['PSRAM free',fmtB(d.psram_free)+' / '+fmtB(d.psram_total)],['SD free',d.sd_total?fmtB(d.sd_free)+' / '+fmtB(d.sd_total):'no card'],['Uptime',uh+'h '+um+'m']];"
        "document.getElementById('sysinfo').innerHTML=rows.map(r=>'<div><span>'+r[0]+'</span><b>'+r[1]+'</b></div>').join('');}catch(e){}}"
        "setInterval(sysRefresh,5000);sysRefresh();"
        "async function timeRefresh(){try{const d=await(await fetch('/time')).json();document.getElementById('clock').textContent=d.time;document.getElementById('ntpState').textContent=d.ntp?(d.synced?'NTP synced':'NTP syncing...'):'NTP off (clips numbered)';document.getElementById('ntpOn').checked=d.ntp;var t=document.getElementById('tz');if(document.activeElement!==t)t.value=d.tz;}catch(e){}}"
        "document.getElementById('ntpOn').onchange=async e=>{await fetch('/ntp?on='+(e.target.checked?1:0),{method:'POST'});setTimeout(timeRefresh,600);};"
        "document.getElementById('tz').onchange=async e=>{await fetch('/ntp?tz='+encodeURIComponent(e.target.value),{method:'POST'});setTimeout(timeRefresh,600);};"
        "setInterval(timeRefresh,2000);timeRefresh();"
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
static httpd_uri_t uri_wifi_list  = { .uri = "/wifi/list", .method = HTTP_GET,  .handler = wifi_list_handler, .user_ctx = NULL };
static httpd_uri_t uri_wifi_add   = { .uri = "/wifi/add",  .method = HTTP_POST, .handler = wifi_add_handler,  .user_ctx = NULL };
static httpd_uri_t uri_wifi_del   = { .uri = "/wifi/del",  .method = HTTP_POST, .handler = wifi_del_handler,  .user_ctx = NULL };
static httpd_uri_t uri_time       = { .uri = "/time",      .method = HTTP_GET,  .handler = time_handler,     .user_ctx = NULL };
static httpd_uri_t uri_ntp        = { .uri = "/ntp",       .method = HTTP_POST, .handler = ntp_handler,      .user_ctx = NULL };
static httpd_uri_t uri_rename     = { .uri = "/rename",    .method = HTTP_POST, .handler = rename_handler,   .user_ctx = NULL };
static httpd_uri_t uri_copy       = { .uri = "/copy",      .method = HTTP_POST, .handler = copy_handler,     .user_ctx = NULL };
static httpd_uri_t uri_sysinfo    = { .uri = "/sysinfo",   .method = HTTP_GET,  .handler = sysinfo_handler,  .user_ctx = NULL };
static httpd_uri_t uri_detect     = { .uri = "/detect",    .method = HTTP_GET,  .handler = detect_handler,   .user_ctx = NULL };
static httpd_uri_t uri_mask_get   = { .uri = "/mask",      .method = HTTP_GET,  .handler = mask_handler,     .user_ctx = NULL };
static httpd_uri_t uri_mask_post  = { .uri = "/mask",      .method = HTTP_POST, .handler = mask_handler,     .user_ctx = NULL };

static httpd_handle_t start_webserver(void) {
    /* Async download worker pool: keeps big/slow downloads off the main worker. */
    dl_slots = xSemaphoreCreateCounting(DL_MAX_CONCURRENT, DL_MAX_CONCURRENT);
    dl_queue = xQueueCreate(DL_MAX_CONCURRENT, sizeof(httpd_req_t *));
    if (dl_slots && dl_queue) {
        for (int i = 0; i < DL_MAX_CONCURRENT; i++)
            xTaskCreate(dl_worker, "dl_wrk", 4096, NULL, 5, NULL);
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 28;
    config.max_resp_headers = 8;
    /* index_handler builds sizable HTML on-stack; 4 KB default is too tight. */
    config.stack_size = 8192;
    /* Room for UI requests + a couple of async downloads at once. */
    config.max_open_sockets = 7;
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
        httpd_register_uri_handler(server, &uri_wifi_list);
        httpd_register_uri_handler(server, &uri_wifi_add);
        httpd_register_uri_handler(server, &uri_wifi_del);
        httpd_register_uri_handler(server, &uri_time);
        httpd_register_uri_handler(server, &uri_ntp);
        httpd_register_uri_handler(server, &uri_rename);
        httpd_register_uri_handler(server, &uri_copy);
        httpd_register_uri_handler(server, &uri_sysinfo);
        httpd_register_uri_handler(server, &uri_detect);
        httpd_register_uri_handler(server, &uri_mask_get);
        httpd_register_uri_handler(server, &uri_mask_post);
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

    ESP_LOGI(TAG, "Saving WiFi network: SSID=%s", ssid);
    wifi_creds_add(ssid, pass);   /* add to the known-networks list (NVS) */
    sd_config_save_all();         /* mirror the whole list to SD */

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

#if CONFIG_BT_NIMBLE_ENABLED
/* ============================ BLE remote / info (NimBLE) ============================
 * Advertises as "XIAO-CAM" with a Nordic-UART-style GATT service so a phone
 * (e.g. nRF Connect) can read status (IP / version / recording) and send
 * commands ("rec on" / "rec off") — works even without knowing the IP.
 * Compiled only when Bluetooth is enabled in sdkconfig (freed for ML otherwise). */
static uint8_t ble_addr_type;
static void ble_advertise(void);

/* NUS UUIDs: service 6e400001-, RX(write) ...0002, TX/status(read+notify) ...0003 */
static const ble_uuid128_t ble_svc_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e);
static const ble_uuid128_t ble_cmd_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e);
static const ble_uuid128_t ble_status_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e);

static int ble_status_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const esp_app_desc_t *app = esp_app_get_description();
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "ip=%s ver=%s cam=%s rec=%s",
                     device_ip[0] ? device_ip : "-", app->version,
                     camera_ready ? "on" : "off", rec_enabled ? "on" : "off");
    return os_mbuf_append(ctxt->om, buf, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int ble_cmd_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char cmd[32] = {0};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
    ble_hs_mbuf_to_flat(ctxt->om, cmd, len, NULL);
    if (strncmp(cmd, "rec on", 6) == 0 || strncmp(cmd, "rec 1", 5) == 0) rec_enabled = true;
    else if (strncmp(cmd, "rec off", 7) == 0 || strncmp(cmd, "rec 0", 5) == 0) rec_enabled = false;
    ESP_LOGI(TAG, "BLE command: '%s'", cmd);
    return 0;
}

static const struct ble_gatt_svc_def ble_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ble_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            { .uuid = &ble_status_uuid.u, .access_cb = ble_status_access, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
            { .uuid = &ble_cmd_uuid.u,    .access_cb = ble_cmd_access,    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
            { 0 },
        },
    },
    { 0 },
};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status != 0) ble_advertise();
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        ble_advertise();
    } else if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ble_advertise();
    }
    return 0;
}

static void ble_advertise(void) {
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"XIAO-CAM";
    fields.name_len = 8;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &advp, ble_gap_event, NULL);
}

static void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_advertise();
    ESP_LOGI(TAG, "BLE advertising as XIAO-CAM");
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void) {
    if (nimble_port_init() != ESP_OK) {
        ESP_LOGW(TAG, "NimBLE init failed");
        return;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(ble_gatt_svcs);
    ble_gatts_add_svcs(ble_gatt_svcs);
    ble_svc_gap_device_name_set("XIAO-CAM");
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
}
#endif /* CONFIG_BT_NIMBLE_ENABLED */

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
    if (camera_ready) {
        ml_ready = (person_detect_init() == 0);
        ESP_LOGI(TAG, "Person detector: %s", ml_ready ? "ready" : "unavailable");
    }
    init_sd_card();
    rec_cfg_load();
    sys_cfg_load();
    if (ml_ready) person_set_thr_pct(motion_pconf);
    wifi_init_sta();
    ntp_apply();   /* start SNTP if enabled (network is up now) */
    init_mdns();
    start_webserver();
    start_stream_webserver();
    /* Loop recorder runs continuously; it idles until enabled via the web UI. */
    xTaskCreate(rec_task, "rec_task", 6144, NULL, 4, NULL);
#if CONFIG_BT_NIMBLE_ENABLED
    ble_init();
#endif
}
