#ifndef GATE_NAVIGATOR_H
#define GATE_NAVIGATOR_H

#include <stdint.h>

extern void gate_navigator_init(void);
extern void gate_navigator_periodic(void);

/** Called by waypoint_navigation when exiting GATE_TRACKING for any reason.
 *  Immediately resets gate_nav to SEARCH (passive), clears gate_tracking. */
extern void gate_navigator_abort(void);

/** 1 while FSM is in TRACK or CROSS state. Read by waypoint_navigation.c. */
extern int gate_tracking;

/** GCS runtime toggle (0=off, 1=on). */
extern uint8_t gate_detection_enabled;

/** Mirrors c_contour_edited's show_threshold_overlay for GCS control. */
extern uint8_t gate_show_overlay;

/** Minimum TTC at gate center [s] required before crossing. GCS-tunable. */
extern float gate_ttc_safe_threshold;

#endif /* GATE_NAVIGATOR_H */
