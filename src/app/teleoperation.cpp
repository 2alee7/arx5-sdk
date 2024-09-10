#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/solver.h"

// P-P (PD) architecture for bilateral teleoperation.

/*
Requires direct controller - server - controller communication on multiple synchronized threads for lowest latency.
*/

// TODO: signal handling

// TODO: initialize 2 controllers

// TODO: initialize connected robots (enable inertia, gravity compensation, friction(?) compensation)

// TODO: get joint state from robot 1, get joint state from robot 2

// TODO: joint-to-joint state comparison (position error, velocity error)

// TODO: define gains for PD controller

// TODO: use performant PD functions (implemented elsewhere if needed) --> FF current + PD current
