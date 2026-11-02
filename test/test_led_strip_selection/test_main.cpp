#include "GeneratedLEDStripImages.h"

#include <unity.h>

namespace {
void testSixPositionSwitchMapping() {
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LEDStripAnimation::Off),
                            static_cast<uint8_t>(LEDStripImages::animationForSpinSwitchPosition(0)));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LEDStripAnimation::TestPattern),
                            static_cast<uint8_t>(LEDStripImages::animationForSpinSwitchPosition(1)));

    for(uint8_t imageIndex = 0; imageIndex < LEDStripImages::GENERATED_USER_IMAGE_COUNT; ++imageIndex) {
        const LEDStripAnimation animation = LEDStripImages::animationForSpinSwitchPosition(imageIndex + 2);
        const uint8_t firstUserImage = static_cast<uint8_t>(LEDStripAnimation::TestPattern) + 1;
        TEST_ASSERT_EQUAL_UINT8(firstUserImage + imageIndex, static_cast<uint8_t>(animation));
    }

    for(uint8_t imageIndex = LEDStripImages::GENERATED_USER_IMAGE_COUNT; imageIndex < 4; ++imageIndex) {
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(LEDStripAnimation::Off),
            static_cast<uint8_t>(LEDStripImages::animationForSpinSwitchPosition(imageIndex + 2)));
    }

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LEDStripAnimation::Off),
                            static_cast<uint8_t>(LEDStripImages::animationForSpinSwitchPosition(6)));
}
} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(testSixPositionSwitchMapping);
    return UNITY_END();
}
