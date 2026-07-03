/* Minimal animated-GIF encoder (writes to a file). Public-domain style,
   adapted from Marcel Rodrigues' gifenc for ESP-IDF (FILE* instead of fd). */
#pragma once
#include <stdint.h>
#include <stdio.h>

typedef struct ge_GIF {
    uint16_t w, h;
    int depth;
    FILE *file;
    int offset;
    int nframes;
    uint8_t *frame, *back;
    uint32_t partial;
    uint8_t buffer[0xFF];
} ge_GIF;

/* palette: 3 * (1<<depth) bytes RGB. loop: 0 = infinite, -1 = no loop. */
ge_GIF *ge_new_gif(const char *fname, uint16_t width, uint16_t height,
                   uint8_t *palette, int depth, int loop);
/* Write gif->frame (indexed pixels) as a frame; delay in centiseconds. */
void ge_add_frame(ge_GIF *gif, uint16_t delay);
void ge_close_gif(ge_GIF *gif);
