/*
 * Copyright (C) 2026
 *
 * This file is part of paparazzi.
 */

#ifndef CYBERZOO_PERIMETER_NAV_H
#define CYBERZOO_PERIMETER_NAV_H

#include "std.h"
#include "math/pprz_algebra_float.h"

extern float cz_perimeter_lookahead;
extern float cz_perimeter_corner_radius;

extern void cyberzoo_perimeter_nav_init(void);
extern void cyberzoo_perimeter_nav_periodic(void);
extern bool cyberzoo_perimeter_get_path_direction(struct FloatVect2 *dir);
extern void ProjectPathToEdge(void);

#endif /* CYBERZOO_PERIMETER_NAV_H */
