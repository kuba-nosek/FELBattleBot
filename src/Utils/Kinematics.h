#pragma once
#include "IAccel.h"
#include <cmath>

namespace Kinematics {

    inline float getCentripetalAcceleration(const AccelData& a1, const AccelData& a2) {
        const float rotX = (a1.xMps2 + a2.xMps2) / 2.0f;
        const float rotY = (a1.yMps2 + a2.yMps2) / 2.0f;
        return std::hypot(rotX, rotY);
    }

    struct TranslationVector { float xMps2; float yMps2; };
    
    inline TranslationVector getTranslationalAcceleration(const AccelData& a1, const AccelData& a2) {
        return {
            (a1.xMps2 - a2.xMps2) / 2.0f,
            (a1.yMps2 - a2.yMps2) / 2.0f
        };
    }

    inline float getAverageZ(const AccelData& a1, const AccelData& a2) {
        return (a1.zMps2 + a2.zMps2) / 2.0f;
    }

} // namespace Kinematics

//different calculation of centripetal acceleration
//should separate tangetial acceleration as well

namespace Kinematics2 {

    constexpr float INV_SQRT2 = 0.70710678118f;
    constexpr float SENSOR_RADIUS_M = 0.015f;

    struct TranslationVector {
        float xMps2;
        float yMps2;
    };

    inline float getCentripetalAcceleration(
        const AccelData& a1,
        const AccelData& a2
    ) {
        const float sx = (a1.xMps2 + a2.xMps2) * 0.5f;
        const float sy = (a1.yMps2 + a2.yMps2) * 0.5f;

        return std::max(0.0f, (sx + sy) * INV_SQRT2);
    }

    inline float getAngularAcceleration(
        const AccelData& a1,
        const AccelData& a2
    ) {
        const float sx = (a1.xMps2 + a2.xMps2) * 0.5f;
        const float sy = (a1.yMps2 + a2.yMps2) * 0.5f;

        const float tangential = (sx - sy) * INV_SQRT2;

        return tangential / SENSOR_RADIUS_M;
    }

    inline TranslationVector getTranslationalAcceleration(
        const AccelData& a1,
        const AccelData& a2
    ) {
        const float tx = (a1.xMps2 - a2.xMps2) * 0.5f;
        const float ty = (a1.yMps2 - a2.yMps2) * 0.5f;

        return {
            (-tx + ty) * INV_SQRT2,
            (-tx - ty) * INV_SQRT2
        };
    }

    inline float getAverageZ(
        const AccelData& a1,
        const AccelData& a2
    ) {
        return (a1.zMps2 + a2.zMps2) * 0.5f;
    }

}
