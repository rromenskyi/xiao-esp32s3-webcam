// C++ wrapper around ESP-DL pedestrian (person) detection, exposed to the C app.
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "pedestrian_detect.hpp"
#include "dl_image_define.hpp"

static const char *TAG = "person";
static PedestrianDetect *g_detect = nullptr;
static uint8_t *g_rgb = nullptr;          // PSRAM decode buffer (1/2-scale rgb565)
static uint8_t *g_rgb888 = nullptr;       // PSRAM RGB888 buffer (full frame)
static const size_t RGB_CAP = 400 * 300 * 2;   // enough for up to UXGA/2 ... capped below
static const size_t RGB888_CAP = 640 * 480 * 3;
static float g_last_score = 0.0f;
static float g_min_score = 0.25f;         // app-side trigger threshold (person present)
static int g_pixfmt = 2;                  // 0=RGB565LE, 1=RGB565BE, 2=RGB888

extern "C" int person_last_score_pct(void) { return (int)(g_last_score * 100.0f); }
extern "C" void person_set_pixfmt(int f) { g_pixfmt = f; }
extern "C" void person_set_thr_pct(int pct) { g_min_score = pct / 100.0f; }
extern "C" int person_thr_pct(void) { return (int)(g_min_score * 100.0f); }

extern "C" int person_detect_init(void) {
    if (g_detect) return 0;
    if (!g_rgb) g_rgb = (uint8_t *)heap_caps_malloc(RGB_CAP, MALLOC_CAP_SPIRAM);
    if (!g_rgb) { ESP_LOGE(TAG, "rgb buffer alloc failed"); return -1; }
    size_t before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    g_detect = new (std::nothrow) PedestrianDetect();
    if (!g_detect) { ESP_LOGE(TAG, "PedestrianDetect alloc failed"); return -1; }
    g_detect->set_score_thr(0.1f);   /* low so we see all candidates; app gates on g_min_score */
    size_t after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "pedestrian detector loaded (internal heap %u -> %u)",
             (unsigned)before, (unsigned)after);
    return 0;
}

// Returns 1 if a person is detected in the JPEG frame, else 0.
extern "C" int person_in_frame(camera_fb_t *fb) {
    if (!g_detect || !fb) return 0;
    dl::image::img_t img = {};
    if (g_pixfmt == 2) {                                  /* full-res RGB888 */
        if ((size_t)(fb->width * fb->height * 3) > RGB888_CAP) return 0;
        if (!g_rgb888) g_rgb888 = (uint8_t *)heap_caps_malloc(RGB888_CAP, MALLOC_CAP_SPIRAM);
        if (!g_rgb888) return 0;
        if (!fmt2rgb888(fb->buf, fb->len, fb->format, g_rgb888)) return 0;
        img.data = g_rgb888;
        img.width = fb->width;
        img.height = fb->height;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
    } else {                                              /* 1/2-scale RGB565 */
        int w = fb->width / 2, h = fb->height / 2;
        if ((size_t)(w * h * 2) > RGB_CAP) return 0;
        if (!g_rgb) g_rgb = (uint8_t *)heap_caps_malloc(RGB_CAP, MALLOC_CAP_SPIRAM);
        if (!g_rgb || !jpg2rgb565(fb->buf, fb->len, g_rgb, JPG_SCALE_2X)) return 0;
        img.data = g_rgb;
        img.width = w;
        img.height = h;
        img.pix_type = (g_pixfmt == 1) ? dl::image::DL_IMAGE_PIX_TYPE_RGB565BE
                                       : dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
    }
    auto &results = g_detect->run(img);
    float best = 0.0f;
    for (const auto &r : results) if (r.score > best) best = r.score;
    g_last_score = best;
    return best >= g_min_score ? 1 : 0;
}
