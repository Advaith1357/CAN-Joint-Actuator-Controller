/* Actuator CAN protocol: frame IDs, payload structs and pack/unpack.
 * See docs/protocol.md for the wire format. */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdint.h>

#define CANP_MAX_NODE        0x0Fu
#define CANP_ID_ESTOP        0x001u
#define CANP_ID_STATUS_BASE  0x080u
#define CANP_ID_TELEM_BASE   0x180u
#define CANP_ID_CMD_BASE     0x200u

#define CANP_CMD_FLAG_CLEAR_FAULT 0x01u

/* Fixed-point scales used on the wire. */
#define CANP_VEL_CMD_SCALE   1e-3f   /* rad/s per LSB, velocity setpoint */
#define CANP_POS_SCALE       1e-4f   /* rad per LSB, position setpoint + telemetry */
#define CANP_VEL_TLM_SCALE   0.02f   /* rad/s per LSB, telemetry velocity */
#define CANP_CUR_SCALE       1e-3f   /* A per LSB, telemetry current */

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum {
    CANP_MODE_IDLE = 0,
    CANP_MODE_VELOCITY = 1,
    CANP_MODE_POSITION = 2,
} canp_mode_t;

typedef enum {
    CANP_KIND_UNKNOWN = 0,
    CANP_KIND_ESTOP,
    CANP_KIND_STATUS,
    CANP_KIND_TELEM,
    CANP_KIND_CMD,
} canp_kind_t;

typedef struct {
    uint8_t mode;     /* canp_mode_t */
    uint8_t flags;
    uint8_t seq;
    int32_t setpoint; /* units depend on mode, see docs/protocol.md */
} canp_cmd_t;

typedef struct {
    int32_t pos;      /* 1e-4 rad */
    int16_t vel;      /* 0.02 rad/s */
    int16_t current;  /* mA */
} canp_telem_t;

typedef struct {
    uint8_t state;
    uint8_t faults;
    uint8_t mode;
    uint8_t last_seq;
    uint32_t uptime_ms;
} canp_status_t;

enum {
    CANP_OK = 0,
    CANP_ERR_ID = -1,    /* frame isn't this message type */
    CANP_ERR_DLC = -2,   /* wrong payload length */
    CANP_ERR_VALUE = -3, /* field out of range (e.g. unknown mode, node > 15) */
};

/* Identify a frame and extract its node ID (0 for broadcast frames). */
canp_kind_t canp_classify(const can_frame_t *f, uint8_t *node);

int canp_encode_cmd(uint8_t node, const canp_cmd_t *cmd, can_frame_t *f);
int canp_decode_cmd(const can_frame_t *f, uint8_t *node, canp_cmd_t *cmd);
int canp_encode_telem(uint8_t node, const canp_telem_t *t, can_frame_t *f);
int canp_decode_telem(const can_frame_t *f, uint8_t *node, canp_telem_t *t);
int canp_encode_status(uint8_t node, const canp_status_t *s, can_frame_t *f);
int canp_decode_status(const can_frame_t *f, uint8_t *node, canp_status_t *s);

/* Physical units -> wire fields, saturating instead of wrapping. */
int16_t canp_sat_i16(float x);
int32_t canp_sat_i32(float x);

#endif
