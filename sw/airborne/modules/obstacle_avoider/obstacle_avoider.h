#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include <stdint.h>
#include "modules/computer_vision/opticflow/inter_thread_data.h"
#include "lib/vision/image.h"

extern float OA_WARNING_TTC;
extern float OA_SAFETY_TTC;
extern float OA_MIN_FPS;
extern float OA_MIN_DIVERGENCE;
extern int OA_IMG_WIDTH;
extern int OA_EDGE_OBSTACLE_THRESHOLD;
extern int OA_FLOOR_MIN_AREA;

extern uint8_t oa_use_opticflow;
extern uint8_t oa_use_edge;
extern uint8_t oa_use_floor;
extern uint8_t oa_use_tree;

#define PRINT(string,...) fprintf(stderr, "[obstacle_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT

void obstacle_avoider_init(void);
void obstacle_avoider_run(void);

#endif