#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <stdint.h>

// navigation states
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  REJOIN_PATH,
  OUT_OF_BOUNDS,
  GO_TO_GATE
};

// settings
extern float safe_max_speed;
extern float obstacle_max_speed;
extern float heading_slew_deg;
extern int16_t cycles_until_rejoin_path;
extern float inner_edge_margin_m;

// state variables
extern enum navigation_state_t navigation_state;
extern int16_t obstacle_free_confidence;

// functions
extern void waypoint_navigation_init(void);
extern void waypoint_navigation_periodic(void);
extern uint8_t nav_state_is_rejoin_path(void);

#endif