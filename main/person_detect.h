#pragma once
#include "esp_camera.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Load the ESP-DL pedestrian detector. Returns 0 on success. */
int person_detect_init(void);

/* 1 if a person is detected in the JPEG frame, else 0. */
int person_in_frame(camera_fb_t *fb);

/* Debug: last max detection score (percent), and switch pixel format (0=565LE,1=565BE). */
int person_last_score_pct(void);
void person_set_pixfmt(int f);
void person_set_thr_pct(int pct);
int person_thr_pct(void);

#ifdef __cplusplus
}
#endif
