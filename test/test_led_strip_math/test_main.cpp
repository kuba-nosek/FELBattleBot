#include "LEDStripMath.h"

#include <limits>
#include <unity.h>

namespace {
constexpr float FLOAT_TOLERANCE = 0.0001f;

void testIntegratesPositiveAngularVelocity() {
    const float angle = LEDStripMath::integrateAngle(0.0f, LEDStripMath::TWO_PI_RADIANS, 250000);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, LEDStripMath::TWO_PI_RADIANS / 4.0f, angle);
}

void testIntegratesNegativeAngularVelocity() {
    const float angle = LEDStripMath::integrateAngle(0.0f, -LEDStripMath::TWO_PI_RADIANS, 250000);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, LEDStripMath::TWO_PI_RADIANS * 0.75f, angle);
}

void testIntegrationUsesWrapSafeElapsedTime() {
    const uint32_t lastUpdateUs = UINT32_MAX - 99;
    const uint32_t currentUs = 50;
    const LEDStripMath::IntegrationState state =
        LEDStripMath::updateIntegration(0.0f, lastUpdateUs, currentUs, 1000.0f, false);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.15f, state.angleRadians);
    TEST_ASSERT_EQUAL_UINT32(currentUs, state.lastUpdateUs);
}

void testResetClearsAngleAndUpdatesTime() {
    const LEDStripMath::IntegrationState state = LEDStripMath::updateIntegration(2.5f, 10, 20, 8.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, state.angleRadians);
    TEST_ASSERT_EQUAL_UINT32(20, state.lastUpdateUs);
}

void testZeroAndNonFiniteVelocityDoNotMoveAngle() {
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.25f, LEDStripMath::integrateAngle(1.25f, 0.0f, 500000));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.25f,
                             LEDStripMath::integrateAngle(1.25f, std::numeric_limits<float>::infinity(), 500000));
}

void testAngleMapsToConfiguredSector() {
    TEST_ASSERT_EQUAL_UINT16(0, LEDStripMath::angleToSector(0.0f, 42));
    TEST_ASSERT_EQUAL_UINT16(10, LEDStripMath::angleToSector(LEDStripMath::TWO_PI_RADIANS / 4.0f, 42));
    TEST_ASSERT_EQUAL_UINT16(31, LEDStripMath::angleToSector(-LEDStripMath::TWO_PI_RADIANS / 4.0f, 42));
}

void testRefreshLimitAndFrameSuppression() {
    TEST_ASSERT_FALSE(LEDStripMath::shouldRefresh(1999, 0, 2000, true));
    TEST_ASSERT_TRUE(LEDStripMath::shouldRefresh(2000, 0, 2000, true));
    TEST_ASSERT_TRUE(LEDStripMath::shouldRefresh(1, UINT32_MAX - 1998, 2000, true));
    TEST_ASSERT_TRUE(LEDStripMath::shouldRefresh(1, 1, 2000, false));

    TEST_ASSERT_FALSE(LEDStripMath::shouldTransmit(true, false));
    TEST_ASSERT_TRUE(LEDStripMath::shouldTransmit(true, true));
    TEST_ASSERT_TRUE(LEDStripMath::shouldTransmit(false, false));
}

void testSpinningTransmissionClearsFullStripBeforeUsingHalf() {
    constexpr uint16_t FULL_LED_COUNT = 14;
    constexpr uint16_t SPIN_LED_COUNT = 7;

    TEST_ASSERT_EQUAL_UINT16(FULL_LED_COUNT,
                             LEDStripMath::transmissionPixelCount(true, false, FULL_LED_COUNT, SPIN_LED_COUNT));
    TEST_ASSERT_EQUAL_UINT16(SPIN_LED_COUNT,
                             LEDStripMath::transmissionPixelCount(true, true, FULL_LED_COUNT, SPIN_LED_COUNT));
    TEST_ASSERT_EQUAL_UINT16(FULL_LED_COUNT,
                             LEDStripMath::transmissionPixelCount(false, true, FULL_LED_COUNT, SPIN_LED_COUNT));
}

void testAnimationChangeDoesNotResetIntegratedPhase() {
    LEDStripMath::IntegrationState state = LEDStripMath::updateIntegration(0.0f, 0, 100000, 2.0f, false);
    state = LEDStripMath::updateIntegration(state.angleRadians, state.lastUpdateUs, 200000, 2.0f, false);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.4f, state.angleRadians);
}
} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(testIntegratesPositiveAngularVelocity);
    RUN_TEST(testIntegratesNegativeAngularVelocity);
    RUN_TEST(testIntegrationUsesWrapSafeElapsedTime);
    RUN_TEST(testResetClearsAngleAndUpdatesTime);
    RUN_TEST(testZeroAndNonFiniteVelocityDoNotMoveAngle);
    RUN_TEST(testAngleMapsToConfiguredSector);
    RUN_TEST(testRefreshLimitAndFrameSuppression);
    RUN_TEST(testSpinningTransmissionClearsFullStripBeforeUsingHalf);
    RUN_TEST(testAnimationChangeDoesNotResetIntegratedPhase);
    return UNITY_END();
}
