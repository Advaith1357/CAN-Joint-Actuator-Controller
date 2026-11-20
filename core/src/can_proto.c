#include "can_proto.h"

#include <string.h>

/* Explicit little-endian packing, so the wire format doesn't depend on the
 * host's endianness or struct padding. */
static void put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *b)
{
    return (uint16_t)(b[0] | (b[1] << 8));
}

static uint32_t get_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

int16_t canp_sat_i16(float x)
{
    if (x >= 32767.0f) return INT16_MAX;
    if (x <= -32768.0f) return INT16_MIN;
    return (int16_t)(x < 0.0f ? x - 0.5f : x + 0.5f);
}

int32_t canp_sat_i32(float x)
{
    /* 2147483520 is the largest float below INT32_MAX. */
    if (x >= 2147483520.0f) return INT32_MAX;
    if (x <= -2147483648.0f) return INT32_MIN;
    return (int32_t)(x < 0.0f ? x - 0.5f : x + 0.5f);
}

static int match_base(uint32_t id, uint32_t base, uint8_t *node)
{
    if (id < base || id > base + CANP_MAX_NODE) return 0;
    *node = (uint8_t)(id - base);
    return 1;
}

canp_kind_t canp_classify(const can_frame_t *f, uint8_t *node)
{
    *node = 0;
    if (f->id == CANP_ID_ESTOP) return CANP_KIND_ESTOP;
    if (match_base(f->id, CANP_ID_STATUS_BASE, node)) return CANP_KIND_STATUS;
    if (match_base(f->id, CANP_ID_TELEM_BASE, node)) return CANP_KIND_TELEM;
    if (match_base(f->id, CANP_ID_CMD_BASE, node)) return CANP_KIND_CMD;
    return CANP_KIND_UNKNOWN;
}

static int begin(uint8_t node, uint32_t base, can_frame_t *f)
{
    if (node > CANP_MAX_NODE) return CANP_ERR_VALUE;
    memset(f, 0, sizeof(*f));
    f->id = base + node;
    f->dlc = 8;
    return CANP_OK;
}

static int check(const can_frame_t *f, uint32_t base, uint8_t *node)
{
    if (!match_base(f->id, base, node)) return CANP_ERR_ID;
    if (f->dlc != 8) return CANP_ERR_DLC;
    return CANP_OK;
}

int canp_encode_cmd(uint8_t node, const canp_cmd_t *cmd, can_frame_t *f)
{
    if (cmd->mode > CANP_MODE_POSITION) return CANP_ERR_VALUE;
    int rc = begin(node, CANP_ID_CMD_BASE, f);
    if (rc) return rc;
    f->data[0] = cmd->mode;
    f->data[1] = cmd->flags;
    f->data[2] = cmd->seq;
    put_u32(&f->data[4], (uint32_t)cmd->setpoint);
    return CANP_OK;
}

int canp_decode_cmd(const can_frame_t *f, uint8_t *node, canp_cmd_t *cmd)
{
    int rc = check(f, CANP_ID_CMD_BASE, node);
    if (rc) return rc;
    if (f->data[0] > CANP_MODE_POSITION) return CANP_ERR_VALUE;
    cmd->mode = f->data[0];
    cmd->flags = f->data[1];
    cmd->seq = f->data[2];
    cmd->setpoint = (int32_t)get_u32(&f->data[4]);
    return CANP_OK;
}

int canp_encode_telem(uint8_t node, const canp_telem_t *t, can_frame_t *f)
{
    int rc = begin(node, CANP_ID_TELEM_BASE, f);
    if (rc) return rc;
    put_u32(&f->data[0], (uint32_t)t->pos);
    put_u16(&f->data[4], (uint16_t)t->vel);
    put_u16(&f->data[6], (uint16_t)t->current);
    return CANP_OK;
}

int canp_decode_telem(const can_frame_t *f, uint8_t *node, canp_telem_t *t)
{
    int rc = check(f, CANP_ID_TELEM_BASE, node);
    if (rc) return rc;
    t->pos = (int32_t)get_u32(&f->data[0]);
    t->vel = (int16_t)get_u16(&f->data[4]);
    t->current = (int16_t)get_u16(&f->data[6]);
    return CANP_OK;
}

int canp_encode_status(uint8_t node, const canp_status_t *s, can_frame_t *f)
{
    int rc = begin(node, CANP_ID_STATUS_BASE, f);
    if (rc) return rc;
    f->data[0] = s->state;
    f->data[1] = s->faults;
    f->data[2] = s->mode;
    f->data[3] = s->last_seq;
    put_u32(&f->data[4], s->uptime_ms);
    return CANP_OK;
}

int canp_decode_status(const can_frame_t *f, uint8_t *node, canp_status_t *s)
{
    int rc = check(f, CANP_ID_STATUS_BASE, node);
    if (rc) return rc;
    s->state = f->data[0];
    s->faults = f->data[1];
    s->mode = f->data[2];
    s->last_seq = f->data[3];
    s->uptime_ms = get_u32(&f->data[4]);
    return CANP_OK;
}
