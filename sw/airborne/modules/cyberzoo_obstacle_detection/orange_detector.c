#include "modules/cyberzoo_obstacle_detection/orange_detector.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT

// define settings for orange detection
float oa_color_count_frac = 0.18f;      // detect threshold (orange fraction)
int32_t color_count = 0;               	// orange color count from color filter for obstacle detection
int16_t orange_free_confidence = 0;  	// certainty that forward direction is safe
uint8_t orange_detected = 0;            // flag to check if orange was detected

// get ABI message from video thread of orange pixel count
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
                            {
                              color_count = quality;
                            }


void orange_detector_init(void) {
    // bind our colorfilter callbacks to receive the color filter outputs
  	AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}


void orange_detector_periodic(void) {
    int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
    if (color_count >= color_count_threshold) {
        orange_detected = 1;
    }
    else {
        orange_detected = 0;
    }

    // update safe confidence using color threshold
    if (color_count < color_count_threshold) {
        orange_free_confidence++;
    } else {
        orange_free_confidence -= 2;
    }

    if (orange_detected) {
        orange_free_confidence = 0;
    }

    Bound(orange_free_confidence, 0, 4);

    AbiSendMsgORANGE_OBSTACLE_DETECTION(ORANGE_OBSTACLE_DETECTION_ID, orange_detected, orange_free_confidence);

}
