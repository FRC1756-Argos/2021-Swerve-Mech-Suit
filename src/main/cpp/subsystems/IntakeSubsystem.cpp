/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"

using argos_lib::talonsrx_config::TalonSRXConfig;

IntakeSubsystem::IntakeSubsystem() : m_intakeMotor(address::motor::intake), m_motorSpeed(0.0) {
  TalonSRXConfig<motorConfig::intake::intakeMotor>(m_intakeMotor, 100_ms);
}

void IntakeSubsystem::Periodic() {
  m_intakeMotor.Set(m_motorSpeed);
}

void IntakeSubsystem::Intake() {
  m_motorSpeed = 1.0;
}

void IntakeSubsystem::Eject() {
  m_motorSpeed = -1.0;
}

void IntakeSubsystem::Stop() {
  m_motorSpeed = 0.0;
}
