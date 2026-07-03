/* Minimal animated-GIF encoder — adapted from Marcel Rodrigues' gifenc
   (https://github.com/lecram/gifenc), public domain, ported to FILE*. */
#include <stdlib.h>
#include <string.h>
#include "gifenc.h"

typedef struct Node {
    uint16_t key;
    struct Node *children[];
} Node;

static Node *new_node(uint16_t key, int degree) {
    Node *node = calloc(1, sizeof(*node) + degree * sizeof(Node *));
    if (node) node->key = key;
    return node;
}

static Node *new_trie(int degree, int *nkeys) {
    Node *root = new_node(0, degree);
    for (*nkeys = 0; *nkeys < degree; (*nkeys)++)
        root->children[*nkeys] = new_node(*nkeys, degree);
    *nkeys += 2;   /* skip clear code and stop code */
    return root;
}

/* Iterative free — the trie can be up to 4096 nodes deep on a solid frame, which
   would blow the task stack if freed recursively. */
static void del_trie(Node *root, int degree) {
    if (!root) return;
    Node **stack = malloc(4098 * sizeof(Node *));
    if (!stack) return;   /* leak rather than crash on OOM */
    int sp = 0;
    stack[sp++] = root;
    while (sp) {
        Node *n = stack[--sp];
        for (int i = 0; i < degree; i++)
            if (n->children[i] && sp < 4098) stack[sp++] = n->children[i];
        free(n);
    }
    free(stack);
}

static void put_num(ge_GIF *gif, uint16_t n) {
    fputc(n & 0xFF, gif->file);
    fputc((n >> 8) & 0xFF, gif->file);
}

static void put_loop(ge_GIF *gif, uint16_t loop) {
    fputc('!', gif->file); fputc(0xFF, gif->file); fputc(11, gif->file);
    fwrite("NETSCAPE2.0", 1, 11, gif->file);
    fputc(3, gif->file); fputc(1, gif->file);
    put_num(gif, loop);
    fputc(0, gif->file);
}

ge_GIF *ge_new_gif(const char *fname, uint16_t width, uint16_t height,
                   uint8_t *palette, int depth, int loop) {
    ge_GIF *gif = calloc(1, sizeof(*gif) + 2 * (size_t)width * height);
    if (!gif) return NULL;
    gif->file = fopen(fname, "wb");
    if (!gif->file) { free(gif); return NULL; }
    gif->w = width; gif->h = height; gif->depth = depth > 1 ? depth : 2;
    gif->frame = (uint8_t *)&gif[1];
    gif->back = &gif->frame[(size_t)width * height];
    fwrite("GIF89a", 1, 6, gif->file);
    put_num(gif, width);
    put_num(gif, height);
    fputc(0xF0 | (depth - 1), gif->file);   /* global color table, 2^depth entries */
    fputc(0, gif->file);                    /* background index */
    fputc(0, gif->file);                    /* pixel aspect ratio */
    fwrite(palette, 1, 3 * (1 << depth), gif->file);
    if (loop >= 0 && loop <= 0xFFFF) put_loop(gif, (uint16_t)loop);
    return gif;
}

static void put_key(ge_GIF *gif, uint16_t key, int key_size) {
    int byte_offset = gif->offset / 8;
    int bit_offset  = gif->offset % 8;
    gif->partial |= ((uint32_t)key) << bit_offset;
    int bits_to_write = bit_offset + key_size;
    while (bits_to_write >= 8) {
        gif->buffer[byte_offset++] = gif->partial & 0xFF;
        if (byte_offset == 0xFF) {
            fputc(0xFF, gif->file);
            fwrite(gif->buffer, 1, 0xFF, gif->file);
            byte_offset = 0;
        }
        gif->partial >>= 8;
        bits_to_write -= 8;
    }
    gif->offset = (gif->offset + key_size) % (0xFF * 8);
}

static void end_key(ge_GIF *gif) {
    int byte_offset = gif->offset / 8;
    if (gif->offset % 8) gif->buffer[byte_offset++] = gif->partial & 0xFF;
    if (byte_offset) { fputc(byte_offset, gif->file); fwrite(gif->buffer, 1, byte_offset, gif->file); }
    fputc(0, gif->file);   /* image block terminator */
    gif->offset = gif->partial = 0;
}

void ge_add_frame(ge_GIF *gif, uint16_t delay) {
    int nkeys, key_size, degree = 1 << gif->depth;

    /* Graphic Control Extension (for the frame delay) */
    if (delay > 0 || gif->nframes) {
        fputc('!', gif->file); fputc(0xF9, gif->file); fputc(4, gif->file);
        fputc(0x04, gif->file);           /* do not dispose */
        put_num(gif, delay);
        fputc(0, gif->file); fputc(0, gif->file);
    }

    /* Image Descriptor */
    fputc(',', gif->file);
    put_num(gif, 0); put_num(gif, 0);
    put_num(gif, gif->w); put_num(gif, gif->h);
    fputc(0, gif->file);

    int min_code_size = gif->depth < 2 ? 2 : gif->depth;
    fputc(min_code_size, gif->file);

    Node *root = new_trie(degree, &nkeys);
    key_size = min_code_size + 1;
    put_key(gif, degree, key_size);        /* clear code */

    Node *node = root;
    size_t npix = (size_t)gif->w * gif->h;
    for (size_t i = 0; i < npix; i++) {
        uint8_t pixel = gif->frame[i] & (degree - 1);
        Node *child = node->children[pixel];
        if (child) {
            node = child;
        } else {
            put_key(gif, node->key, key_size);
            if (nkeys < 0x1000) {
                if (nkeys == (1 << key_size)) key_size++;
                node->children[pixel] = new_node(nkeys++, degree);
            } else {
                put_key(gif, degree, key_size);        /* clear code */
                del_trie(root, degree);
                root = new_trie(degree, &nkeys);
                key_size = min_code_size + 1;
            }
            node = root->children[pixel];
        }
    }
    put_key(gif, node->key, key_size);
    put_key(gif, degree + 1, key_size);    /* stop code */
    end_key(gif);
    del_trie(root, degree);
    gif->nframes++;
}

void ge_close_gif(ge_GIF *gif) {
    fputc(';', gif->file);                 /* trailer */
    fclose(gif->file);
    free(gif);
}
