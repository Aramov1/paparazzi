/*
 * viz_select.h
 *
 * Shared visualization selector for the RTP debug stream.
 * All camera modules include this header and check VIZ_ACTIVE
 * before drawing anything into the image buffer.
 * Detection logic in every module runs unconditionally.
 *
 * Values:
 *   0 = clean stream  (no overlay drawn by any module)
 *   1 = edge detection overlay
 *   2 = contour / tree detector overlay
 *   3 = optic flow overlay  (sets opticflow[0].show_flow)
 *   4 = gate detection overlay  (reserved, not yet implemented)
 *
 * The extern "C" guards are required because edge_detection.cpp
 * is compiled as C++ — without them the C++ compiler mangles
 * VIZ_ACTIVE to a different symbol and the comparison always fails.
 */
#ifndef VIZ_SELECT_H
#define VIZ_SELECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_VIZ_SELECT
extern uint8_t VIZ_ACTIVE;
#else
#define VIZ_ACTIVE 0
#endif

#define VIZ_NONE     0
#define VIZ_EDGE     1
#define VIZ_CONTOUR  2
#define VIZ_OPFLOW   3
#define VIZ_GATE     4

#ifdef __cplusplus
}
#endif

#endif /* VIZ_SELECT_H */
