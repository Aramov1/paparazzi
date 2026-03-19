#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

#include <stdint.h>

// settings
extern float oa_color_count_frac;
extern float oa_clear_color_count_frac;
extern float oa_safe_max_speed;
extern float oa_heading_slew_deg;
extern int16_t oa_rejoin_forward_cycles;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif
