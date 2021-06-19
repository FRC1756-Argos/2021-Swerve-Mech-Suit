#pragma once

#include "ctre/Phoenix.h"
#include <units/time.h>
#include "compileTimeMemberCheck.h"

HAS_MEMBER(inverted)
HAS_MEMBER(sensorPhase)
HAS_MEMBER(neutralMode)
HAS_MEMBER(voltCompSat)
HAS_MEMBER(remoteFilter0_addr)
HAS_MEMBER(remoteFilter0_type)
HAS_MEMBER(pid0_selectedSensor)
HAS_MEMBER(pid0_kP)
HAS_MEMBER(pid0_kI)
HAS_MEMBER(pid0_kD)
HAS_MEMBER(pid0_kF)
HAS_MEMBER(pid0_iZone)
HAS_MEMBER(pid0_allowableError)

template<typename T>
bool FalconConfig(WPI_TalonFX& motorController, units::millisecond_t configTimeout ) {
  TalonFXConfiguration config;
  auto timeout = configTimeout.to<int>();

  motorController.ConfigFactoryDefault(timeout);

  if constexpr(has_inverted<T>{}) {
    motorController.SetInverted(T::inverted);
  }
  if constexpr(has_sensorPhase<T>{}) {
    motorController.SetSensorPhase(T::sensorPhase);
  }
  if constexpr(has_neutralMode<T>{}) {
    motorController.SetNeutralMode(T::neutralMode);
  }
  if constexpr(has_voltCompSat<T>{}) {
    config.voltageCompSaturation = T::voltCompSat;
    motorController.EnableVoltageCompensation(true);
  } else {
    motorController.EnableVoltageCompensation(false);
  }
  if constexpr(has_remoteFilter0_addr<T>{} && has_remoteFilter0_type<T>{}) {
    ctre::phoenix::motorcontrol::can::FilterConfiguration filterConfig;
    filterConfig.remoteSensorDeviceID = T::remoteFilter0_addr;
    filterConfig.remoteSensorSource = T::remoteFilter0_type;
    config.remoteFilter0 = filterConfig;
  }
  if constexpr(has_pid0_selectedSensor<T>{}) {
    config.primaryPID.selectedFeedbackSensor = T::pid0_selectedSensor;
  }
  if constexpr(has_pid0_kP<T>{}) {
    config.slot0.kP = T::pid0_kP;
  }
  if constexpr(has_pid0_kI<T>{}) {
    config.slot0.kI = T::pid0_kI;
  }
  if constexpr(has_pid0_kD<T>{}) {
    config.slot0.kD = T::pid0_kD;
  }
  if constexpr(has_pid0_kF<T>{}) {
    config.slot0.kF = T::pid0_kF;
  }
  if constexpr(has_pid0_iZone<T>{}) {
    config.slot0.integralZone = T::pid0_iZone;
  }
  if constexpr(has_pid0_allowableError<T>{}) {
    config.slot0.allowableClosedloopError = T::pid0_allowableError;
  }

  return 0 != motorController.ConfigAllSettings(config, timeout);
}
