#ifndef ROBOARM_MOTION_H
#define ROBOARM_MOTION_H

#include "zf_common_headfile.h"

void roboarm_init(void);

void arm_reset(void);
void arm_relax(void);
void arm_reset_to_prepare(void);
void arm_prepare_to_grip(void);
void arm_grip_to_shot(void);
void arm_shot_to_reset(void);

#endif // ROBOARM_MOTION_H
