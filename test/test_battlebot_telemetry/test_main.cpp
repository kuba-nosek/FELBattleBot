#include "BattlebotTelemetry.h"

#include <cmath>
#include <cstdint>
#include <unity.h>

namespace {

void testEngineeringValueEncoding() {
    TEST_ASSERT_EQUAL_INT16(123, BattlebotTelemetry::encodeAccelerationCentiG(1.234f));
    TEST_ASSERT_EQUAL_INT16(-10000, BattlebotTelemetry::encodeAccelerationCentiG(-120.0f));
    TEST_ASSERT_EQUAL_INT16(0, BattlebotTelemetry::encodeAccelerationCentiG(NAN));
    TEST_ASSERT_EQUAL_UINT16(17321, BattlebotTelemetry::encodePeakMagnitudeCentiG(200.0f));
    TEST_ASSERT_EQUAL_INT16(-1234, BattlebotTelemetry::encodeRpm(-1234.4f));
    TEST_ASSERT_EQUAL_INT16(32767, BattlebotTelemetry::encodeRpm(50000.0f));
}

void testFrameLayoutAndCrc() {
    BattlebotTelemetry::Snapshot snapshot{};
    snapshot.rpm = -1234;
    snapshot.validMask = 0x7F;
    snapshot.accelerationCentiG[0][0] = 123;
    snapshot.accelerationCentiG[0][1] = -456;
    snapshot.accelerationCentiG[0][2] = 789;
    snapshot.accelerationCentiG[1][0] = -10000;
    snapshot.accelerationCentiG[1][1] = 0;
    snapshot.accelerationCentiG[1][2] = 10000;
    snapshot.peakMagnitudeCentiG[0] = 10001;
    snapshot.peakMagnitudeCentiG[1] = 17321;

    uint8_t frame[BattlebotTelemetry::FRAME_SIZE]{};
    const size_t size = BattlebotTelemetry::buildFrame(frame, sizeof(frame), 0x5A, snapshot);

    const uint8_t expected[] = {
        0xC8, 0x18, 0x80, 0xF3, 0x01, 0x5A, 0x7F, 0xFB, 0x2E, 0x00, 0x7B, 0xFE, 0x38,
        0x03, 0x15, 0xD8, 0xF0, 0x00, 0x00, 0x27, 0x10, 0x27, 0x11, 0x43, 0xA9, 0x7C,
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), size);
    // cppcheck-suppress cstyleCast
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, sizeof(expected));
}

void testSmallOrNullOutputIsRejected() {
    BattlebotTelemetry::Snapshot snapshot{};
    uint8_t shortFrame[BattlebotTelemetry::FRAME_SIZE - 1]{};

    TEST_ASSERT_EQUAL_UINT32(0, BattlebotTelemetry::buildFrame(shortFrame, sizeof(shortFrame), 0, snapshot));
    TEST_ASSERT_EQUAL_UINT32(0, BattlebotTelemetry::buildFrame(nullptr, BattlebotTelemetry::FRAME_SIZE, 0, snapshot));
}

} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(testEngineeringValueEncoding);
    RUN_TEST(testFrameLayoutAndCrc);
    RUN_TEST(testSmallOrNullOutputIsRejected);
    return UNITY_END();
}
