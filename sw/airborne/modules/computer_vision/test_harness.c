/*
 * test_harness.c
 *
 * Loads JPEG/PNG frames from disk, converts to UYVY,
 * runs find_contour(), saves annotated overlay PNGs,
 * and prints a detection log to stdout.
 *
 * Build:
 *   gcc -DNPS -DDEBUG_CONTOUR -O2 \
 *       test_harness.c c_contour_edited.c \
 *       -lm -o gate_test
 *
 * Run:
 *   ./gate_test images/*.jpg 2>&1 | tee results.log
 */

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "c_contour_edited.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */
static uint8_t clamp_u8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v;
}

static void rgb_to_yuv(uint8_t r, uint8_t g, uint8_t b,
                        uint8_t *y, uint8_t *u, uint8_t *v)
{
    *y = clamp_u8(( 66*r + 129*g +  25*b + 128) / 256 + 16);
    *u = clamp_u8((-38*r -  74*g + 112*b + 128) / 256 + 128);
    *v = clamp_u8((112*r -  94*g -  18*b + 128) / 256 + 128);
}

/* -------------------------------------------------------------------------
 * RGB → UYVY
 * Width is rounded down to even. Caller frees returned buffer.
 * ------------------------------------------------------------------------- */
static uint8_t *rgb_to_uyvy(const uint8_t *rgb, int width, int height,
                              int *out_width)
{
    int w = width & ~1;
    *out_width = w;
    uint8_t *uyvy = malloc((size_t)w * height * 2);
    if (!uyvy) return NULL;

    for (int row = 0; row < height; row++) {
        for (int col = 0; col < w; col += 2) {
            const uint8_t *p0 = rgb + (row * width + col)     * 3;
            const uint8_t *p1 = rgb + (row * width + col + 1) * 3;

            uint8_t y0, u0, v0, y1, u1, v1;
            rgb_to_yuv(p0[0], p0[1], p0[2], &y0, &u0, &v0);
            rgb_to_yuv(p1[0], p1[1], p1[2], &y1, &u1, &v1);

            int i = row * w + col;
            uyvy[2*i]     = (u0 + u1) / 2;  /* U  */
            uyvy[2*i + 1] = y0;               /* Y0 */
            uyvy[2*i + 2] = (v0 + v1) / 2;  /* V  */
            uyvy[2*i + 3] = y1;               /* Y1 */
        }
    }
    return uyvy;
}

/* -------------------------------------------------------------------------
 * UYVY → RGB  (for saving overlay PNG)
 * ------------------------------------------------------------------------- */
static uint8_t *uyvy_to_rgb(const uint8_t *uyvy, int width, int height)
{
    uint8_t *rgb = malloc((size_t)width * height * 3);
    if (!rgb) return NULL;

    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col += 2) {
            int i = row * width + col;
            uint8_t u  = uyvy[2*i];
            uint8_t y0 = uyvy[2*i + 1];
            uint8_t v  = uyvy[2*i + 2];
            uint8_t y1 = uyvy[2*i + 3];

            int c0 = (int)y0 - 16,  c1 = (int)y1 - 16;
            int d  = (int)u  - 128, e  = (int)v  - 128;

            uint8_t *o0 = rgb + (row * width + col)     * 3;
            uint8_t *o1 = rgb + (row * width + col + 1) * 3;

            o0[0] = clamp_u8((298*c0           + 409*e + 128) >> 8);
            o0[1] = clamp_u8((298*c0 - 100*d - 208*e + 128) >> 8);
            o0[2] = clamp_u8((298*c0 + 516*d           + 128) >> 8);

            o1[0] = clamp_u8((298*c1           + 409*e + 128) >> 8);
            o1[1] = clamp_u8((298*c1 - 100*d - 208*e + 128) >> 8);
            o1[2] = clamp_u8((298*c1 + 516*d           + 128) >> 8);
        }
    }
    return rgb;
}

/* -------------------------------------------------------------------------
 * Build output filename:  "images/frame_001.jpg" → "out_frame_001.png"
 * ------------------------------------------------------------------------- */
static void make_out_name(const char *in_path, char *out_path, size_t sz)
{
    const char *base = strrchr(in_path, '/');
    base = base ? base + 1 : in_path;
    const char *dot  = strrchr(base, '.');
    size_t stem_len  = dot ? (size_t)(dot - base) : strlen(base);
    snprintf(out_path, sz, "out_%.*s.png", (int)stem_len, base);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s image1.jpg [image2.jpg ...]\n", argv[0]);
        return 1;
    }

    show_threshold_overlay = 1;
    gate_tracking          = 0;   /* SEARCH mode */
    contour_reset_tracking();

    int total = 0, detected = 0;

    for (int a = 1; a < argc; a++) {
        const char *path = argv[a];

        /* load image ---------------------------------------------------- */
        int w, h, ch;
        uint8_t *rgb = stbi_load(path, &w, &h, &ch, 3);
        if (!rgb) {
            fprintf(stderr, "[SKIP] Cannot load: %s\n", path);
            continue;
        }

        /* size guard — detector hard-limits at 320×520 */
        if (w > 320 || h > 520) {
            fprintf(stderr,
                    "[WARN] %s is %dx%d — exceeds 320×520, skipping\n",
                    path, w, h);
            stbi_image_free(rgb);
            continue;
        }

        /* convert to UYVY ----------------------------------------------- */
        int uyvy_w;
        uint8_t *uyvy = rgb_to_uyvy(rgb, w, h, &uyvy_w);
        stbi_image_free(rgb);
        if (!uyvy) { fprintf(stderr, "[ERR] out of memory\n"); continue; }

        /* optional: print how many mask pixels were set
         * (uncomment to debug color classifier) */
        /*
        {
            extern uint8_t mask[];   // not accessible here — add inside find_contour instead
        }
        */

        /* run detector -------------------------------------------------- */
        cont_est.gate_detected = 0;
        find_contour((char *)uyvy, uyvy_w, h);
        total++;

        printf("=== %s (%dx%d) ===\n", path, uyvy_w, h);
        if (cont_est.gate_detected) {
            detected++;
            printf("  GATE DETECTED  dist=%.2f  lat=%.3f  vert=%.3f  area=%.0f\n",
                   cont_est.contour_d_x,
                   cont_est.contour_d_z,
                   cont_est.contour_d_y,
                   cont_est.contour_area);
        } else {
            printf("  no gate\n");
        }

        /* save annotated overlay PNG ------------------------------------ */
        char out_name[512];
        make_out_name(path, out_name, sizeof(out_name));
        uint8_t *out_rgb = uyvy_to_rgb(uyvy, uyvy_w, h);
        if (out_rgb) {
            if (stbi_write_png(out_name, uyvy_w, h, 3, out_rgb, uyvy_w * 3))
                printf("  overlay  → %s\n", out_name);
            else
                fprintf(stderr, "  [ERR] failed to write %s\n", out_name);
            free(out_rgb);
        }
        free(uyvy);
    }

    printf("\n=== Summary: %d / %d frames with gate detected ===\n",
           detected, total);
    return 0;
}