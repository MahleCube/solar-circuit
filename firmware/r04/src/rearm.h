#ifndef C07_REARM_H
#define C07_REARM_H
#include <stdbool.h>
#include <stdint.h>
#include "board_config.h"

/* State belongs to the applied GPIO, independently of the supervisor clock. */
typedef struct { bool high; uint64_t raised_us; } c07_rearm;
static inline bool c07_rearm_level(const c07_rearm *r, bool request,
                                  bool pv_enabled, uint64_t now) {
    if (!pv_enabled) return false; /* Stop cancels a pulse immediately. */
    return request || (r->high && (now<r->raised_us || now-r->raised_us<C07_REARM_US));
}
static inline void c07_rearm_written(c07_rearm *r, bool high, uint64_t completed_us) {
    if (high&&!r->high) r->raised_us=completed_us;
    r->high=high;
}
#endif
