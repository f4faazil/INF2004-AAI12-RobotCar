#ifndef MOCK_MUTEX_PORT_H
#define MOCK_MUTEX_PORT_H
/* Fail exactly the Nth upcoming mutex lock attempt, N >= 1. */
void mock_mutex_fail_after(int n);
#endif
