/*
 * test_support.h
 * A tiny assert-style harness for the host-side tests (no external deps).
 */
#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <stdio.h>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        g_checks++;                                                       \
        if (!(cond)) {                                                    \
            g_failures++;                                                 \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                 \
    } while (0)

#define CHECK_INT(expected, actual)                                       \
    do {                                                                  \
        long exp_ = (long)(expected);                                     \
        long act_ = (long)(actual);                                       \
        g_checks++;                                                       \
        if (exp_ != act_) {                                               \
            g_failures++;                                                 \
            printf("    FAIL %s:%d: %s: expected %ld, got %ld\n",         \
                   __FILE__, __LINE__, #actual, exp_, act_);              \
        }                                                                 \
    } while (0)

#define RUN(test)                                                         \
    do {                                                                  \
        int before_ = g_failures;                                         \
        test();                                                           \
        printf("%s  %s\n", (g_failures == before_) ? "PASS" : "FAIL",     \
               #test);                                                    \
    } while (0)

#define TEST_REPORT()                                                     \
    (printf("%d checks, %d failures\n", g_checks, g_failures),            \
     (g_failures == 0) ? 0 : 1)

#endif /* TEST_SUPPORT_H */
