// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() : m_driverController(address::joystick::driver),
                                   m_autonomousCommand(&m_exampleSubsystem),
                                   m_driveLonSpeedMap(controllerMap::driveLongSpeed),
                                   m_driveLatSpeedMap(controllerMap::driveLatSpeed),
                                   m_driveRotSpeedMap(controllerMap::driveRotSpeed)
{
  // Initialize all of your commands and subsystems here

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.SwerveDrive(
            m_driveLonSpeedMap(m_driverController.GetY(frc::GenericHID::kLeftHand)),
            m_driveLatSpeedMap(m_driverController.GetX(frc::GenericHID::kLeftHand)),
            m_driveRotSpeedMap(m_driverController.GetX(frc::GenericHID::kRightHand)));
      },
      {&m_drive}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
