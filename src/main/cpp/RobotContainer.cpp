// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc2/command/RunCommand.h>
#include <networktables/NetworkTableInstance.h>

RobotContainer::RobotContainer() : m_controllers(address::joystick::driver, address::joystick::secondary),
                                   m_autonomousCommand(&m_exampleSubsystem),
                                   m_driveLonSpeedMap(controllerMap::driveLongSpeed),
                                   m_driveLatSpeedMap(controllerMap::driveLatSpeed),
                                   m_driveRotSpeedMap(controllerMap::driveRotSpeed)
{
  // Initialize all of your commands and subsystems here

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.SwerveDrive(
            m_driveLonSpeedMap(m_controllers.driverController().GetY(frc::GenericHID::kLeftHand)),
            m_driveLatSpeedMap(m_controllers.driverController().GetX(frc::GenericHID::kLeftHand)),
            m_driveRotSpeedMap(m_controllers.driverController().GetX(frc::GenericHID::kRightHand)));
      },
      {&m_drive}));

  // Configure the button bindings
  ConfigureButtonBindings();

  // Configure network tables
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  ntTable->SetPersistent(ntKeys::subsystemDrive::homePosition::turnFrontLeft);
  ntTable->SetPersistent(ntKeys::subsystemDrive::homePosition::turnFrontRight);
  ntTable->SetPersistent(ntKeys::subsystemDrive::homePosition::turnRearRight);
  ntTable->SetPersistent(ntKeys::subsystemDrive::homePosition::turnRearLeft);

  // Set controller configs
  m_controllers.driverController().SetButtonDebounce(ArgosLib::XboxController::Button::kBack, {1_s, 0_ms});
  m_controllers.driverController().SetButtonDebounce(ArgosLib::XboxController::Button::kStart, {1_s, 0_ms});
  m_controllers.operatorController().SetButtonDebounce(ArgosLib::XboxController::Button::kBack, {1_s, 0_ms});
  m_controllers.operatorController().SetButtonDebounce(ArgosLib::XboxController::Button::kStart, {1_s, 0_ms});

  m_controllers.driverController().SetButtonDebounce(ArgosLib::XboxController::Button::kBumperLeft, {500_ms, 0_ms});
  m_controllers.driverController().SetButtonDebounce(ArgosLib::XboxController::Button::kBumperRight, {500_ms, 0_ms});
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  m_triggerHomeCombo.WhenActive(&m_homeSwerveModulesCommand);
  (m_driverTriggerSwapCombo || m_operatorTriggerSwapCombo).WhileActiveOnce(SwapControllersCommand(&m_controllers));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
