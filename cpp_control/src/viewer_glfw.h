#pragma once

#include <mujoco/mujoco.h>

typedef void (*ControlStepFn)(const mjModel* m, mjData* d, void* user);

// Opens a MuJoCo render window and calls control_step before each mj_step.
// Returns 0 on normal close, non-zero on setup failure.
int run_mujoco_viewer(mjModel* m, mjData* d, ControlStepFn control_step, void* user);
