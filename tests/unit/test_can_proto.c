#include "can_proto.h"
#include "test.h"

static void test_cmd_roundtrip(void)
{
    canp_cmd_t in = {.mode = CANP_MODE_POSITION, .flags = 0, .seq = 42, .setpoint = -123456}, out = {0};
    can_frame_t f;
    uint8_t node = 0xFF;
    CHECK(canp_encode_cmd(5, &in, &f) == CANP_OK);
    CHECK(f.id == 0x205);
    CHECK(f.dlc == 8);
    CHECK(canp_decode_cmd(&f, &node, &out) == CANP_OK);
    CHECK(node == 5);
    CHECK(out.mode == in.mode);
    CHECK(out.seq == 42);
    CHECK(out.setpoint == -123456);
}

static void test_wire_format_is_little_endian(void)
{
    canp_cmd_t in = {.mode = CANP_MODE_VELOCITY, .flags = 1, .seq = 7, .setpoint = 0x01020304};
    can_frame_t f;
    canp_encode_cmd(0, &in, &f);
    const uint8_t expect[8] = {1, 1, 7, 0, 0x04, 0x03, 0x02, 0x01};
    for (int i = 0; i < 8; i++) CHECK(f.data[i] == expect[i]);
}

static void test_telem_and_status_roundtrip(void)
{
    canp_telem_t t = {.pos = -5, .vel = -32768, .current = 1234}, t2;
    canp_status_t s = {.state = 2, .faults = 0x05, .mode = 1, .last_seq = 255, .uptime_ms = 4000000000u}, s2;
    can_frame_t f;
    uint8_t node;
    CHECK(canp_encode_telem(15, &t, &f) == CANP_OK);
    CHECK(f.id == 0x18F);
    CHECK(canp_decode_telem(&f, &node, &t2) == CANP_OK);
    CHECK(node == 15 && t2.pos == -5 && t2.vel == -32768 && t2.current == 1234);
    CHECK(canp_encode_status(3, &s, &f) == CANP_OK);
    CHECK(canp_decode_status(&f, &node, &s2) == CANP_OK);
    CHECK(node == 3 && s2.state == 2 && s2.faults == 0x05 && s2.last_seq == 255 && s2.uptime_ms == 4000000000u);
}

static void test_rejects_malformed(void)
{
    canp_cmd_t cmd = {.mode = CANP_MODE_VELOCITY}, out;
    can_frame_t f;
    uint8_t node;
    CHECK(canp_encode_cmd(16, &cmd, &f) == CANP_ERR_VALUE); /* node out of range */

    canp_encode_cmd(1, &cmd, &f);
    f.dlc = 4;
    CHECK(canp_decode_cmd(&f, &node, &out) == CANP_ERR_DLC);

    canp_encode_cmd(1, &cmd, &f);
    f.data[0] = 9; /* unknown mode */
    CHECK(canp_decode_cmd(&f, &node, &out) == CANP_ERR_VALUE);

    canp_encode_cmd(1, &cmd, &f);
    f.id = 0x181; /* a telemetry ID */
    CHECK(canp_decode_cmd(&f, &node, &out) == CANP_ERR_ID);

    cmd.mode = 3;
    CHECK(canp_encode_cmd(1, &cmd, &f) == CANP_ERR_VALUE);
}

static void test_classify(void)
{
    can_frame_t f = {.id = CANP_ID_ESTOP};
    uint8_t node;
    CHECK(canp_classify(&f, &node) == CANP_KIND_ESTOP);
    f.id = 0x20A;
    CHECK(canp_classify(&f, &node) == CANP_KIND_CMD && node == 10);
    f.id = 0x08F;
    CHECK(canp_classify(&f, &node) == CANP_KIND_STATUS && node == 15);
    f.id = 0x210; /* just past the command range */
    CHECK(canp_classify(&f, &node) == CANP_KIND_UNKNOWN);
    f.id = 0x7F0;
    CHECK(canp_classify(&f, &node) == CANP_KIND_UNKNOWN);
}

static void test_saturating_conversions(void)
{
    CHECK(canp_sat_i16(1e9f) == 32767);
    CHECK(canp_sat_i16(-1e9f) == -32768);
    CHECK(canp_sat_i16(2.5f) == 3);
    CHECK(canp_sat_i16(-2.5f) == -3);
    CHECK(canp_sat_i32(1e12f) == INT32_MAX);
    CHECK(canp_sat_i32(-1e12f) == INT32_MIN);
}

void run_can_proto_tests(void)
{
    RUN(test_cmd_roundtrip);
    RUN(test_wire_format_is_little_endian);
    RUN(test_telem_and_status_roundtrip);
    RUN(test_rejects_malformed);
    RUN(test_classify);
    RUN(test_saturating_conversions);
}
