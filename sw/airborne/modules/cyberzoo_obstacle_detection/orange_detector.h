#ifndef PAPARAZZI_ORANGE_DETECTOR_H
#define PAPARAZZI_ORANGE_DETECTOR_H

#include <stdint.h>

extern float oa_color_count_frac;

extern void orange_detector_init(void);
extern void orange_detector_periodic(void);

#endif