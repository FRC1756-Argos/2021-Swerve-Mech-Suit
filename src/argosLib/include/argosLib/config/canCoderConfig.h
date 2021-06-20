/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include "compileTimeMemberCheck.h"
#include "ctre/Phoenix.h"

HAS_MEMBER(direction)
HAS_MEMBER(initMode)
HAS_MEMBER(range)
HAS_MEMBER(magOffset)

template <typename T>
bool CanCoderConfig(CANCoder& encoder, units::millisecond_t configTimeout) {
  ctre::phoenix::sensors::CANCoderConfiguration config;
  auto timeout = configTimeout.to<int>();

  if constexpr (has_direction<T>{}) {
    config.sensorDirection = T::direction;
  }
  if constexpr (has_initMode<T>{}) {
    config.initializationStrategy = T::initMode;
  }
  if constexpr (has_range<T>{}) {
    config.absoluteSensorRange = T::range;
  }
  if constexpr (has_magOffset<T>{}) {
    config.magnetOffsetDegrees = T::magOffset;
  }

  return 0 != encoder.ConfigAllSettings(config, timeout);
}
