#include "BattlebotTelemetry.h"
#include "DriveModeType.h"

#include <cmath>
#include <cstdint>
#include <unity.h>

namespace {

void testTelemetryFieldsAreNullable() {
    BattlebotTelemetry::TelemetryData telemetry{};

    TEST_ASSERT_FALSE(telemetry.mode.hasValue());
    TEST_ASSERT_EQUAL_UINT8(0, telemetry.mode.value());
    TEST_ASSERT_FALSE(telemetry.rpm.hasValue());

    telemetry.mode = DriveModeType::Spin;
    telemetry.rpm = -1234;
    TEST_ASSERT_TRUE(telemetry.mode.hasValue());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriveModeType::Spin), telemetry.mode.value());
    TEST_ASSERT_TRUE(telemetry.rpm.hasValue());
    TEST_ASSERT_EQUAL_INT16(-1234, telemetry.rpm.value());

    telemetry.rpm = nullptr;
    TEST_ASSERT_FALSE(telemetry.rpm.hasValue());
    TEST_ASSERT_EQUAL_INT16(0, telemetry.rpm.value());

    telemetry.clear();
    TEST_ASSERT_FALSE(telemetry.mode.hasValue());
    TEST_ASSERT_EQUAL_UINT8(0, telemetry.mode.value());
}

void testNullableFloatField() {
    BattlebotTelemetry::TelemetryField<float> field{};

    TEST_ASSERT_FALSE(field.hasValue());
    field = 1.25f;
    TEST_ASSERT_TRUE(field.hasValue());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.25f, field.value());
    field.reset();
    TEST_ASSERT_FALSE(field.hasValue());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, field.value());
}

void testDirectFieldAssignment() {
    BattlebotTelemetry::TelemetryData telemetry{};

    telemetry.accel1X = 1.2345f;
    TEST_ASSERT_TRUE(telemetry.accel1X.hasValue());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.2345f, telemetry.accel1X.value());

    telemetry.accel1Y = NAN;
    TEST_ASSERT_TRUE(telemetry.accel1Y.hasValue());
    TEST_ASSERT_TRUE(std::isnan(telemetry.accel1Y.value()));

    telemetry.accel1Z = INFINITY;
    TEST_ASSERT_TRUE(telemetry.accel1Z.hasValue());
    TEST_ASSERT_TRUE(std::isinf(telemetry.accel1Z.value()));

    telemetry.rpm = 1234.75f;
    TEST_ASSERT_TRUE(telemetry.rpm.hasValue());
    TEST_ASSERT_EQUAL_INT16(1234, telemetry.rpm.value());
}

void testFrameLayoutAndCrc() {
    BattlebotTelemetry::TelemetryData telemetry{};
    telemetry.mode = DriveModeType::Spin;
    telemetry.rpm = -1234;
    telemetry.accel1X = 1.5f;
    telemetry.accel1Y = 42.0f;
    telemetry.accel1Y = nullptr;
    telemetry.accel1Z = -2.25f;
    telemetry.accel2Y = 1000.0f;

    uint8_t frame[BattlebotTelemetry::FRAME_SIZE]{};
    const size_t size = BattlebotTelemetry::buildFrame(frame, sizeof(frame), 0x5A, telemetry);

    const uint8_t expected[] = {
        0xC8, 0x24, 0x80, 0xF3, 0x02, 0x5A, 0x00, 0x00, 0x00, 0x57, 0x03, 0xFB, 0x2E,
        0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x10, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x44, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78,
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), size);
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), BattlebotTelemetry::FRAME_SIZE);
    // cppcheck-suppress cstyleCast
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, sizeof(expected));
}

void testSmallOrNullOutputIsRejected() {
    BattlebotTelemetry::TelemetryData telemetry{};
    uint8_t shortFrame[BattlebotTelemetry::FRAME_SIZE - 1]{};

    TEST_ASSERT_EQUAL_UINT32(0, BattlebotTelemetry::buildFrame(shortFrame, sizeof(shortFrame), 0, telemetry));
    TEST_ASSERT_EQUAL_UINT32(0, BattlebotTelemetry::buildFrame(nullptr, BattlebotTelemetry::FRAME_SIZE, 0, telemetry));
}

} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(testTelemetryFieldsAreNullable);
    RUN_TEST(testNullableFloatField);
    RUN_TEST(testDirectFieldAssignment);
    RUN_TEST(testFrameLayoutAndCrc);
    RUN_TEST(testSmallOrNullOutputIsRejected);
    return UNITY_END();
}
