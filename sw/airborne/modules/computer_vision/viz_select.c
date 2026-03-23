/*
 * viz_select.c
 *
 * Defines the shared VIZ_ACTIVE variable.
 * Default 0 = clean stream on boot.
 * Change from GCS via the viz_select dl_setting.
 */
#include "viz_select.h"
 
uint8_t VIZ_ACTIVE = VIZ_NONE;