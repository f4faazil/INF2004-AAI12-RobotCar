#include "telemetry_rtos.h"
#include "telemetry_tkernel_port.h"
#include "hardware/timer.h"

/* Initialized once before starting other tasks: no concurrent init/reset. */
static telemetry_inbox_t shared_inbox;
static int inbox_mutex_id;
static bool inbox_ready;

static bool try_mutex(void *context)
{
    const int id = *(const int *)context;
    /* TMO_POL is used inside the isolated T-Kernel port: never wait. */
    return telemetry_tkernel_mutex_try_lock(id) != 0;
}

static void unlock_mutex(void *context)
{
    const int id = *(const int *)context;
    telemetry_tkernel_mutex_unlock(id);
}

bool telemetry_rtos_init(void)
{
    int id;

    if (inbox_ready) return false;   /* Do not reset an active inbox. */
    id = telemetry_tkernel_mutex_create();
    if (id <= 0) return false;

    inbox_mutex_id = id;
    if (!telemetry_inbox_init(&shared_inbox, try_mutex, unlock_mutex,
                              &inbox_mutex_id)) {
        telemetry_tkernel_mutex_delete(id);
        inbox_mutex_id = 0;
        return false;
    }
    inbox_ready = true;
    return true;
}

telemetry_inbox_result_t telemetry_rtos_submit(const uint8_t *bytes,
                                                size_t length,
                                                uint32_t created_ms)
{
    if (!inbox_ready) return TELEMETRY_INBOX_INVALID;
    return telemetry_inbox_submit(&shared_inbox, bytes, length, created_ms);
}

telemetry_inbox_result_t telemetry_rtos_peek(uint32_t now_ms,
                                              uint8_t *out,
                                              size_t out_capacity,
                                              size_t *out_length)
{
    if (!inbox_ready) return TELEMETRY_INBOX_INVALID;
    return telemetry_inbox_peek(&shared_inbox, now_ms, out, out_capacity,
                                out_length);
}

telemetry_inbox_result_t telemetry_rtos_commit(void)
{
    if (!inbox_ready) return TELEMETRY_INBOX_INVALID;
    return telemetry_inbox_commit(&shared_inbox);
}

telemetry_inbox_result_t telemetry_rtos_snapshot(telemetry_inbox_stats_t *out)
{
    if (!inbox_ready) return TELEMETRY_INBOX_INVALID;
    return telemetry_inbox_snapshot(&shared_inbox, out);
}

/* Narrow stats ABI: app_main.c never needs stddef.h or telemetry_inbox.h.
 * An unavailable mutex gives a skipped sample rather than blocking. */
int telemetry_rtos_stats_copy(unsigned int *queued,
                              unsigned int *dropped_full,
                              unsigned int *dropped_stale)
{
    telemetry_inbox_stats_t snapshot;
    if (queued == NULL || dropped_full == NULL || dropped_stale == NULL) {
        return 0;
    }
    if (telemetry_rtos_snapshot(&snapshot) != TELEMETRY_INBOX_OK) {
        return 0;
    }
    *queued = (unsigned int)snapshot.queued;
    *dropped_full = (unsigned int)snapshot.dropped_full;
    *dropped_stale = (unsigned int)snapshot.dropped_stale;
    return 1;
}

/* Small startup ABI for app_main.c, which already includes tk/tkernel.h.
 * Using this int-returning entrypoint avoids stddef.h/size_t header clashes. */
int telemetry_rtos_boot_init(void)
{
    return telemetry_rtos_init() ? 1 : 0;
}

/* Called by MQTT owner after usermain initialized the mutex/inbox, and
 * before any concurrent telemetry producers are started. */
bool telemetry_rtos_ready(void)
{
    return inbox_ready;
}

int telemetry_robot_submit(const unsigned char *bytes,
                           unsigned int length)
{
    const uint32_t now_ms = (uint32_t)(time_us_64() / 1000ULL);

    return (int)telemetry_rtos_submit(bytes, (size_t)length, now_ms);
}
