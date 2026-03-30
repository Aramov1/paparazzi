/*
 * @file modules/flip_bebop/guidance_flip.c
 * @author Jose Cunha
 *  Modification of firmwares/rotorcraft/guidance/guidance_flip.c, tuned to the Parrot Bebop drone. Provides open-loop
 *  guidance for making a rolling flip. Also includes modifications to be better utilised as a module which can be run
 *  at the click of a button defined in the flightplan by defining the following block:
 *
 *  <block name="FLIP">
      <call_once fun="guidance_flip_enter()"/>
      <while cond="!guidance_flip_finished()"/>
      <exception cond="guidance_flip_finished()" deroute="Standby"/>
    </block>
 *
 * After finishing flip, it restores the heading that was set prior to the flip, as well as the prior autopilot mode.
 * Use it with caution!
 */

#ifndef GUIDANCE_FLIP_H
#define GUIDANCE_FLIP_H

#include "std.h"

void guidance_flip_init(void);
void guidance_flip_enter(void);
bool guidance_flip_run(void);
bool guidance_flip_finished(void);
bool guidance_flip_active(void);

#endif /* GUIDANCE_FLIP_H */
