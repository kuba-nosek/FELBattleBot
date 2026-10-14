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
