#pragma once

#include <functional>
#include <Units/time.h>

namespace ArgosLib {

struct VibrationStatus {
  double intensityLeft = 0.0;
  double intensityRight = 0.0;
};

using VibrationModel = std::function<VibrationStatus()>;

VibrationModel VibrationOff();
VibrationModel VibrationConstant(double);
VibrationModel VibrationConstant(double, double);
VibrationModel VibrationSyncPulse(units::millisecond_t, double, double=0.0);
VibrationModel VibrationSyncWave(units::millisecond_t, double, double=0.0);
VibrationModel VibrationAlternatePulse(units::millisecond_t, double, double=0.0);
VibrationModel VibrationAlternateWave(units::millisecond_t, double, double=0.0);

} // namespace ArgosLib
