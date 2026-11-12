#pragma once
#include "IAccel.h"
#include <cmath>

namespace Kinematics {

    inline float getCentripetalAcceleration(const AccelData& a1, const AccelData& a2) {
        float X = (a1.xMps2 + a2.xMps2) / 2.0f;
        float Y = (a1.yMps2 + a2.yMps2) / 2.0f;
        float t = std::hypot(X, Y);
        // assuming 45deg angle between axis running through both accels and each accel axis
        float alfa;
        if (Y >= X) alfa = 45 - std::acos(Y/t);
        else alfa = 45 - std::acos(X/t);
        float ad = t*std::cos(alfa);
        return ad;
    }

    inline float getTangentialAcceleration(const AccelData& a1, const AccelData& a2) {
        float X = (a1.xMps2 + a2.xMps2) / 2.0f;
        float Y = (a1.yMps2 + a2.yMps2) / 2.0f;
        float t = std::hypot(X, Y);
        // assuming 45deg angle between axis running through both accels and each accel axis
        float alfa;
        if (Y >= X) alfa = 45 - std::acos(Y/t);
        else alfa = 45 - std::acos(X/t);
        float at = t*std::sin(alfa);
        return at;
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
