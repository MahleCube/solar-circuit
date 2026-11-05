#ifndef C07_COMMISSIONING_H
#define C07_COMMISSIONING_H
#include "commutation.h"

/* C07 circuit identification is not physical commissioning. Populate this
 * profile from bounded sensor, gate, component, startup and thermal evidence.
 * A nominal datasheet value or simulation PASS does not set qualified=true.
 * The checked-in profile intentionally inhibits ALL power-path enables. */
typedef struct {
    c07_commutation_config commutation;
} c07_commissioning_profile;

#define C07_CAL_GAIN {1,1,1,1,1,1,1,1}
#define C07_CAL_OFFSET {0,0,0,0,0,0,0,0}

#ifdef C07_SIMULATION_PROFILE
/* Private validation-only include. Never enabled by the production project. */
#include "simulation_profile.h"
#else
static const c07_commissioning_profile c07_commissioning={0};
#endif
#endif
