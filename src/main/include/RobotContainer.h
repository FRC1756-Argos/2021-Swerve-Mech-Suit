#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/ExampleCommand.h"
#include "argosLib/commands/SwapControllersCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "argosLib/subsystems/SwappableControllersSubsystem.h"
#include "argosLib/general/interpolation.h"
#include "argosLib/controller/XboxController.h"
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

  // Subsystems
  DriveSubsystem m_drive;
  ExampleSubsystem m_exampleSubsystem;
  SwappableControllersSubsystem m_controllers;

  // Commands
  ExampleCommand m_autonomousCommand;

  interpolationMap<decltype(controllerMap::driveLongSpeed.front().inVal), controllerMap::driveLongSpeed.size()> m_driveLonSpeedMap;
  interpolationMap<decltype(controllerMap::driveLatSpeed.front().inVal), controllerMap::driveLatSpeed.size()> m_driveLatSpeedMap;
  interpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()> m_driveRotSpeedMap;

  void ConfigureButtonBindings();
};
