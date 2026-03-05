#ifndef OPTICFLOW_ATTEMPT1_H
#define OPTICFLOW_ATTEMPT1_H

#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"

extern struct opticflow_t opticflow[];
extern struct opticflow_result_t opticflow_result[];

extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern struct image_t *opticflow_module_calc(struct image_t *img, uint8_t camera_id);

#endif