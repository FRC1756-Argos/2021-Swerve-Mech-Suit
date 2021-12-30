/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "argos_lib/commands/swap_controllers_command.h"
#include "argos_lib/controller/xbox_controller.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"
#include "commands/ExampleCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

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

  // Subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  ExampleSubsystem m_exampleSubsystem;
  argos_lib::SwappableControllersSubsystem m_controllers;

  // Commands
  ExampleCommand m_autonomousCommand;

  argos_lib::InterpolationMap<decltype(controllerMap::driveLongSpeed.front().inVal),
                              controllerMap::driveLongSpeed.size()>
      m_driveLonSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveLatSpeed.front().inVal), controllerMap::driveLatSpeed.size()>
      m_driveLatSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeedMap;

  void ConfigureButtonBindings();
};
