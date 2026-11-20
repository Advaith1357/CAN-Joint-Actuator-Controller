/* Host simulator entry point.
 *
 *   actuator_sim [--node N] [--rx-port P] [--tx-port Q]
 *
 * Listens for CAN frames on UDP 127.0.0.1:P and sends to 127.0.0.1:Q. */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "app.h"
#include "hal_sim.h"
#include "osal.h"

static void on_signal(int sig)
{
    (void)sig;
    _exit(0);
}

static long arg_long(int argc, char **argv, const char *name, long dflt)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], name) == 0) return strtol(argv[i + 1], NULL, 0);
    }
    return dflt;
}

int main(int argc, char **argv)
{
    const long node = arg_long(argc, argv, "--node", 1);
    const long rx_port = arg_long(argc, argv, "--rx-port", 47001);
    const long tx_port = arg_long(argc, argv, "--tx-port", 47002);
    if (node < 0 || node > (long)CANP_MAX_NODE || rx_port <= 0 || rx_port > 65535 || tx_port <= 0 || tx_port > 65535) {
        fprintf(stderr, "usage: %s [--node 0-15] [--rx-port P] [--tx-port Q]\n", argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    const motor_params_t motor = motor_default_params();
    if (hal_sim_init((uint16_t)rx_port, (uint16_t)tx_port, &motor) != 0) {
        perror("hal_sim_init");
        return 1;
    }

    app_config_t cfg = {.node_id = (uint8_t)node, .ctl = ctl_default_config()};
    if (app_init(&cfg) != 0) {
        fprintf(stderr, "app_init failed\n");
        return 1;
    }
    fprintf(stderr, "actuator_sim: node %ld, rx udp:%ld, tx udp:%ld\n", node, rx_port, tx_port);
    osal_start();
    return 0;
}
