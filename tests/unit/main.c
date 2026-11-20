#include <stdio.h>

#include "test.h"

int g_test_failures = 0;
int g_test_checks = 0;

void run_pid_tests(void);
void run_can_proto_tests(void);
void run_controller_tests(void);

int main(void)
{
    run_pid_tests();
    run_can_proto_tests();
    run_controller_tests();
    printf("\n%d checks, %d failures\n", g_test_checks, g_test_failures);
    return g_test_failures ? 1 : 0;
}
