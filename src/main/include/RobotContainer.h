// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/ExampleCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "general/interpolation.h"
#include "controller/XboxController.h"
#include "Constants.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  ArgosLib::XboxController m_driverController;

  // Subsystems
  DriveSubsystem m_drive;
  ExampleSubsystem m_exampleSubsystem;

  // Commands
  ExampleCommand m_autonomousCommand;
  frc2::InstantCommand m_homeSwerveModulesCommand{[this](){m_drive.Home(0_deg);}, {&m_drive}};

  // Triggers
  frc2::Trigger m_triggerHomeCombo{[this](){return m_driverController.GetDebouncedButton({ArgosLib::XboxController::Button::kBack,
                                                                                          ArgosLib::XboxController::Button::kStart});}};

  interpolationMap<decltype(controllerMap::driveLongSpeed.front().inVal), controllerMap::driveLongSpeed.size()> m_driveLonSpeedMap;
  interpolationMap<decltype(controllerMap::driveLatSpeed.front().inVal), controllerMap::driveLatSpeed.size()> m_driveLatSpeedMap;
  interpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()> m_driveRotSpeedMap;

  void ConfigureButtonBindings();
};
