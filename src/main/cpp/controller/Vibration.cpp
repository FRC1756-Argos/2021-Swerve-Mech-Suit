#include <chrono>
#include "controller/Vibration.h"

using namespace ArgosLib;

VibrationModel VibrationOff() {
  return [](){ return VibrationStatus{0.0, 0.0}; };
}

VibrationModel VibrationConstant(double intensity) {
  return [intensity](){ return VibrationStatus{intensity, intensity}; };
}

VibrationModel VibrationConstant(double intensityLeft, double intensityRight) {
  return [intensityLeft, intensityRight](){ return VibrationStatus{intensityLeft, intensityRight}; };
}

VibrationModel VibrationSyncPulse(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff](){ const auto periodTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() % msPeriod;
                                                  const auto vibrationIntensity = periodTime < (msPeriod / 2) ? intensityOn : intensityOff;
                                                  return VibrationStatus{vibrationIntensity, vibrationIntensity}; };
}

VibrationModel VibrationAlternatePulse(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff](){ const auto periodTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() % msPeriod;
                                                  const auto vibrationState = periodTime < (msPeriod / 2);
                                                  return VibrationStatus{vibrationState ? intensityOn : intensityOff,
                                                                         vibrationState ? intensityOff : intensityOn}; };
}

VibrationModel VibrationSyncWave(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff](){ const auto periodTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() % msPeriod;
                                                  const auto periodProgress = static_cast<double>(periodTime) / msPeriod;
                                                  const auto vibrationIntensity = std::cos(M_PI * 2.0 * periodProgress) / 2 + 0.5;
                                                  const auto outputIntensity = intensityOff + vibrationIntensity *  (intensityOn - intensityOff);
                                                  return VibrationStatus{outputIntensity, outputIntensity}; };
}

VibrationModel VibrationAlternateWave(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff](){ const auto periodTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() % msPeriod;
                                                  const auto periodProgress = static_cast<double>(periodTime) / msPeriod;
                                                  const auto vibrationIntensityLeft = std::cos(M_PI * 2.0 * periodProgress) / 2 + 0.5;
                                                  const auto vibrationIntensityRight = 1.0 - vibrationIntensityLeft;
                                                  return VibrationStatus{intensityOff + vibrationIntensityLeft *  (intensityOn - intensityOff),
                                                                         intensityOff + vibrationIntensityRight *  (intensityOn - intensityOff)}; };
}