#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

#include <stdint.h>

// 0=SAFE 1=OBSTACLE_FOUND 2=SEARCH_FOR_SAFE_HEADING 3=REJOIN_PATH 4=OUT_OF_BOUNDS
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  REJOIN_PATH,
  OUT_OF_BOUNDS
};
extern enum navigation_state_t navigation_state;
extern int16_t obstacle_free_confidence;

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
