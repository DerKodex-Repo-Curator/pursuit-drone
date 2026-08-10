/* Cross-language interoperability.
 *
 * Everything else verifies the two ends of the link *separately*: the Python
 * tests check Python packs correctly, the firmware tests check C++ parses
 * correctly, and gen_protocol_header.py asserts the struct sizes match.
 *
 * None of that proves they actually talk to each other. Struct sizes can
 * agree while field ORDER differs; endianness can be right in one direction
 * and wrong in the other; a CRC can be computed over a different byte range
 * on each side and still be self-consistent. Every one of those bugs passes
 * both test suites and fails on the first flight.
 *
 * So these are real bytes, emitted by the actual Python vision stack
 * (tools/gen_interop_vectors.py) and parsed here by the actual firmware
 * code path the aircraft will run.
 */
#include <unity.h>

#include <cmath>
#include <cstring>

#include "flight_state.h"
#include "interop_vectors.h"
#include "pursuit_link.h"

void test_python_packets_pass_firmware_validation(void)
{
    for (int i = 0; i < INTEROP_VECTOR_COUNT; i++) {
        const InteropVector &v = INTEROP_VECTORS[i];
        TEST_ASSERT_TRUE_MESSAGE(
            pursuit_cmd_valid(v.bytes, sizeof(v.bytes)), v.name);
    }
}

void test_fields_survive_the_crossing(void)
{
    /* The test that catches a field-order or endianness mismatch: sizes can
     * agree perfectly while every value lands in the wrong slot. */
    for (int i = 0; i < INTEROP_VECTOR_COUNT; i++) {
        const InteropVector &v = INTEROP_VECTORS[i];
        pursuit_cmd_t c;
        std::memcpy(&c, v.bytes, sizeof(c));

        TEST_ASSERT_EQUAL_UINT16_MESSAGE(PURSUIT_CMD_MAGIC, c.magic, v.name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(PURSUIT_PROTOCOL_VERSION, c.version,
                                        v.name);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(v.seq, c.seq, v.name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(v.flags, c.flags, v.name);
        /* float32 on the wire, so compare with tolerance rather than
         * exactly — Python's doubles are wider than what is transmitted. */
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, v.roll, c.roll, v.name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, v.pitch, c.pitch, v.name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3, v.yaw_rate, c.yaw_rate, v.name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, v.alt_sp, c.alt_sp, v.name);
    }
}

void test_crc_agrees_across_languages(void)
{
    /* A CRC computed over a different byte range on each side is
     * self-consistent within each language and rejects everything across
     * the link. Recomputing Python's CRC with the C implementation is the
     * only way to see it. */
    for (int i = 0; i < INTEROP_VECTOR_COUNT; i++) {
        const InteropVector &v = INTEROP_VECTORS[i];
        pursuit_cmd_t c;
        std::memcpy(&c, v.bytes, sizeof(c));
        const uint16_t ours = pursuit_crc16(v.bytes, sizeof(c) - 2);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(c.crc, ours, v.name);
    }
}

void test_a_real_session_drives_the_state_machine(void)
{
    /* Replay the captured packets in order through the actual flight state
     * machine, exactly as the aircraft would receive them. */
    FlightState fs;
    fs.sensors_ready();

    uint32_t t = 1000;
    const InteropVector &hover = INTEROP_VECTORS[0];   /* armed, alt hold  */
    TEST_ASSERT_TRUE(fs.on_packet(hover.bytes, sizeof(hover.bytes), t));
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_ARMED, fs.state());

    t += 33;
    const InteropVector &locked = INTEROP_VECTORS[1];  /* climbing, locked */
    TEST_ASSERT_TRUE(fs.on_packet(locked.bytes, sizeof(locked.bytes), t));
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, fs.setpoint().yaw_rate_dps);
    TEST_ASSERT_TRUE(fs.setpoint().locked);

    t += 33;
    const InteropVector &fail = INTEROP_VECTORS[2];    /* target lost      */
    TEST_ASSERT_TRUE(fs.on_packet(fail.bytes, sizeof(fail.bytes), t));
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FAILSAFE, fs.state());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fs.setpoint().yaw_rate_dps);

    t += 33;
    const InteropVector &off = INTEROP_VECTORS[3];     /* deadman released */
    TEST_ASSERT_TRUE(fs.on_packet(off.bytes, sizeof(off.bytes), t));
    TEST_ASSERT_FALSE(fs.motors_enabled());

    TEST_ASSERT_EQUAL_UINT32(4, fs.accepted());
    TEST_ASSERT_EQUAL_UINT32(0, fs.rejected());
}

void test_python_extremes_are_clamped_not_obeyed(void)
{
    /* The vision stack clamps before sending, but the firmware must not rely
     * on that — a hostile or buggy sender is the same as a corrupted one. */
    FlightState fs;
    fs.sensors_ready();
    const InteropVector &ext = INTEROP_VECTORS[5];     /* 900 deg, 9000 dps */
    TEST_ASSERT_TRUE(fs.on_packet(ext.bytes, sizeof(ext.bytes), 1000));
    TEST_ASSERT_TRUE(fabsf(fs.setpoint().roll_deg) <= 25.001f);
    TEST_ASSERT_TRUE(fabsf(fs.setpoint().pitch_deg) <= 25.001f);
    TEST_ASSERT_TRUE(fabsf(fs.setpoint().yaw_rate_dps) <= 180.001f);
}

void test_a_duplicate_command_is_not_reapplied(void)
{
    /* The Python link sends each command several times so a burst of WiFi
     * loss cannot gap it. A non-strict sequence comparison would treat every
     * copy as fresh and re-apply the same setpoint — which was a real bug
     * here until redundancy exposed it. */
    FlightState fs;
    fs.sensors_ready();
    const InteropVector &v = INTEROP_VECTORS[0];

    TEST_ASSERT_TRUE(fs.on_packet(v.bytes, sizeof(v.bytes), 1000));
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_FALSE(fs.on_packet(v.bytes, sizeof(v.bytes), 1000 + i));

    TEST_ASSERT_EQUAL_UINT32(1, fs.accepted());
    TEST_ASSERT_EQUAL_UINT32(4, fs.stale());
}

void test_corrupting_a_python_packet_is_caught(void)
{
    for (int i = 0; i < INTEROP_VECTOR_COUNT; i++) {
        const InteropVector &v = INTEROP_VECTORS[i];
        for (size_t b = 0; b < sizeof(v.bytes); b++) {
            uint8_t buf[30];
            std::memcpy(buf, v.bytes, sizeof(buf));
            buf[b] ^= 0x40;
            TEST_ASSERT_FALSE_MESSAGE(pursuit_cmd_valid(buf, sizeof(buf)),
                                      v.name);
        }
    }
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_python_packets_pass_firmware_validation);
    RUN_TEST(test_fields_survive_the_crossing);
    RUN_TEST(test_crc_agrees_across_languages);
    RUN_TEST(test_a_real_session_drives_the_state_machine);
    RUN_TEST(test_python_extremes_are_clamped_not_obeyed);
    RUN_TEST(test_a_duplicate_command_is_not_reapplied);
    RUN_TEST(test_corrupting_a_python_packet_is_caught);
    return UNITY_END();
}
