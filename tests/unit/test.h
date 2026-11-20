/* Minimal test framework: no dependencies, works on host and in CI. */
#ifndef TEST_H
#define TEST_H

#include <math.h>
#include <stdio.h>

extern int g_test_failures;
extern int g_test_checks;

#define CHECK(cond)                                                           \
    do {                                                                      \
        g_test_checks++;                                                      \
        if (!(cond)) {                                                        \
            g_test_failures++;                                                \
            fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                     \
    do {                                                                          \
        double a_ = (double)(a), b_ = (double)(b), t_ = (double)(tol);            \
        g_test_checks++;                                                          \
        if (fabs(a_ - b_) > t_) {                                                 \
            g_test_failures++;                                                    \
            fprintf(stderr, "  FAIL %s:%d: %s = %g, expected %g +/- %g\n",        \
                    __FILE__, __LINE__, #a, a_, b_, t_);                        \
        }                                                                         \
    } while (0)

#define RUN(fn)                                       \
    do {                                              \
        int before_ = g_test_failures;                \
        fn();                                         \
        printf("%s %s\n", g_test_failures == before_ ? "ok  " : "FAIL", #fn); \
    } while (0)

#endif
