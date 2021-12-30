/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

RobotContainer::RobotContainer()
    : m_controllers(address::joystick::driver, address::joystick::secondary)
    , m_autonomousCommand(&m_exampleSubsystem)
    , m_driveLonSpeedMap(controllerMap::driveLongSpeed)
    , m_driveLatSpeedMap(controllerMap::driveLatSpeed)
    , m_driveRotSpeedMap(controllerMap::driveRotSpeed) {
  // Initialize all of your commands and subsystems here

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.SwerveDrive(m_driveLonSpeedMap(m_controllers.DriverController().GetY(frc::GenericHID::kLeftHand)),
                            m_driveLatSpeedMap(m_controllers.DriverController().GetX(frc::GenericHID::kLeftHand)),
                            m_driveRotSpeedMap(m_controllers.DriverController().GetX(frc::GenericHID::kRightHand)));
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
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperLeft, {500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperRight, {500_ms, 0_ms});

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kLeftTrigger, {500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kRightTrigger, {500_ms, 0_ms});
}

void RobotContainer::ConfigureButtonBindings() {
  // Triggers
  frc2::Trigger triggerHomeCombo{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBumperLeft, argos_lib::XboxController::Button::kBumperRight});
  }};
  frc2::Trigger driverTriggerSwapCombo{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};
  frc2::Trigger operatorTriggerSwapCombo{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};
  frc2::Trigger triggerResetFieldOrientation{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kLeftTrigger, argos_lib::XboxController::Button::kRightTrigger});
  }};
  auto robotCentricDriveTrigger =
      (frc2::Trigger{[this]() {
         return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperRight);
       }} &&
       frc2::Trigger{[this]() {
         return !m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperLeft);
       }});
  auto intakeForwardTrigger =
      (frc2::Trigger{[this]() {
         return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
       }} &&
       frc2::Trigger{[this]() {
         return !m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
       }});
  auto intakeReverseTrigger =
      (frc2::Trigger{[this]() {
         return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
       }} &&
       frc2::Trigger{[this]() {
         return !m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
       }});

  // Configure your button bindings here
  robotCentricDriveTrigger.WhenActive([this]() { m_drive.SetControlMode(DriveSubsystem::ControlMode::robotCentric); },
                                      {&m_drive});
  robotCentricDriveTrigger.WhenInactive([this]() { m_drive.SetControlMode(DriveSubsystem::ControlMode::fieldCentric); },
                                        {&m_drive});
  intakeForwardTrigger.WhenActive([this]() { m_intake.Intake(); }, {&m_intake});
  intakeForwardTrigger.WhenInactive([this]() { m_intake.Stop(); }, {&m_intake});
  intakeReverseTrigger.WhenActive([this]() { m_intake.Eject(); }, {&m_intake});
  intakeReverseTrigger.WhenInactive([this]() { m_intake.Stop(); }, {&m_intake});
  triggerHomeCombo.WhenActive([this]() { m_drive.Home(0_deg); }, {&m_drive});
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileActiveOnce(argos_lib::SwapControllersCommand(&m_controllers));
  triggerResetFieldOrientation.WhenActive([this]() { m_drive.SetFieldOrientation(0_deg); }, {&m_drive});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
