#include "mock_mutex_port.h"
#include "telemetry_tkernel_port.h"
#include <assert.h>
static int created;
static int held;
static int attempts;
static int fail_at;
int telemetry_tkernel_mutex_create(void) { assert(!created); created=1; return 42; }
int telemetry_tkernel_mutex_try_lock(int id) {
    assert(id==42 && created);
    ++attempts;
    if (attempts==fail_at) { fail_at=0; return 0; }
    if (held) return 0;
    held=1;
    return 1;
}
void telemetry_tkernel_mutex_unlock(int id) { assert(id==42 && held); held=0; }
void telemetry_tkernel_mutex_delete(int id) { assert(id==42); created=0; held=0; }
void mock_mutex_fail_after(int n) { assert(n>=1); fail_at=attempts+n; }
