/* Host HAL: the "CAN bus" is UDP on localhost and the motor is motor_model.
 *
 * The plant advances by exactly one control period each time the control task
 * calls hal_set_voltage(), so simulated physics stays consistent even when the
 * host OS delays a thread. */
#include "hal_sim.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "app.h"
#include "hal.h"

static int g_sock = -1;
static struct sockaddr_in g_tx_addr;
static motor_model_t g_motor;
static pthread_mutex_t g_motor_mtx = PTHREAD_MUTEX_INITIALIZER;

int hal_sim_init(uint16_t rx_port, uint16_t tx_port, const motor_params_t *motor)
{
    motor_model_init(&g_motor, motor);

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) return -1;
    struct sockaddr_in rx = {.sin_family = AF_INET, .sin_port = htons(rx_port)};
    rx.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(g_sock, (struct sockaddr *)&rx, sizeof(rx)) != 0) return -1;

    memset(&g_tx_addr, 0, sizeof(g_tx_addr));
    g_tx_addr.sin_family = AF_INET;
    g_tx_addr.sin_port = htons(tx_port);
    g_tx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return 0;
}

int hal_can_send(const can_frame_t *f)
{
    uint8_t buf[SIM_UDP_FRAME_LEN] = {0};
    buf[0] = (uint8_t)f->id;
    buf[1] = (uint8_t)(f->id >> 8);
    buf[2] = (uint8_t)(f->id >> 16);
    buf[3] = (uint8_t)(f->id >> 24);
    buf[4] = f->dlc;
    memcpy(&buf[8], f->data, 8);
    ssize_t n = sendto(g_sock, buf, sizeof(buf), 0, (struct sockaddr *)&g_tx_addr, sizeof(g_tx_addr));
    return n == (ssize_t)sizeof(buf) ? 0 : -1;
}

int hal_can_recv(can_frame_t *f, uint32_t timeout_ms)
{
    struct pollfd p = {.fd = g_sock, .events = POLLIN};
    int r = poll(&p, 1, (int)timeout_ms);
    if (r == 0) return 1;
    if (r < 0) return -1;

    uint8_t buf[SIM_UDP_FRAME_LEN];
    ssize_t n = recv(g_sock, buf, sizeof(buf), 0);
    if (n != (ssize_t)sizeof(buf) || buf[4] > 8) return -1; /* runt or bad DLC: drop */
    f->id = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    f->dlc = buf[4];
    memcpy(f->data, &buf[8], 8);
    return 0;
}

int32_t hal_encoder_count(void)
{
    pthread_mutex_lock(&g_motor_mtx);
    int32_t c = motor_model_encoder(&g_motor);
    pthread_mutex_unlock(&g_motor_mtx);
    return c;
}

float hal_current_a(void)
{
    pthread_mutex_lock(&g_motor_mtx);
    float i = g_motor.i;
    pthread_mutex_unlock(&g_motor_mtx);
    return i;
}

void hal_set_voltage(float volts)
{
    pthread_mutex_lock(&g_motor_mtx);
    motor_model_step(&g_motor, volts, (float)APP_CONTROL_PERIOD_MS * 1e-3f);
    pthread_mutex_unlock(&g_motor_mtx);
}

void hal_on_unhandled_frame(const can_frame_t *f)
{
    if (f->id == SIM_ID_SET_LOAD && f->dlc >= 4) {
        int32_t raw = (int32_t)((uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) |
                                ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24));
        pthread_mutex_lock(&g_motor_mtx);
        g_motor.load_torque = (float)raw * 1e-4f;
        pthread_mutex_unlock(&g_motor_mtx);
    }
}
