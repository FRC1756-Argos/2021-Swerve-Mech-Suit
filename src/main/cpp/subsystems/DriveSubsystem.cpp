/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"
#include "argos_lib/config/cancoder_config.h"
#include "argos_lib/config/falcon_config.h"

using argos_lib::cancoder_config::CanCoderConfig;
using argos_lib::falcon_config::FalconConfig;
using argos_lib::swerve::Optimize;

DriveSubsystem::DriveSubsystem(std::unique_ptr<argos_lib::swerve::SwerveHomeStorageInterface> homingStorage)
    : m_motorDriveFrontLeft(address::motor::frontLeftDrive)
    , m_motorDriveFrontRight(address::motor::frontRightDrive)
    , m_motorDriveRearRight(address::motor::rearRightDrive)
    , m_motorDriveRearLeft(address::motor::rearLeftDrive)
    , m_motorTurnFrontLeft(address::motor::frontLeftTurn)
    , m_motorTurnFrontRight(address::motor::frontRightTurn)
    , m_motorTurnRearRight(address::motor::rearRightTurn)
    , m_motorTurnRearLeft(address::motor::rearLeftTurn)
    , m_encoderTurnFrontLeft(address::encoder::frontLeftTurn)
    , m_encoderTurnFrontRight(address::encoder::frontRightTurn)
    , m_encoderTurnRearRight(address::encoder::rearRightTurn)
    , m_encoderTurnRearLeft(address::encoder::rearLeftTurn)
    , m_IMU(sensorConfig::drive::IMU::yawAxis,
            sensorConfig::drive::IMU::port,
            sensorConfig::drive::IMU::calTime.to<uint16_t>())
    , m_fieldOrientationOffset(0_deg)
    , m_pHomingStorage(std::move(homingStorage))
    , m_activeControlMode(ControlMode::fieldCentric) {
  // Set lever arms for each module
  frc::Translation2d leverArmFrontLeft{measureUp::chassis::length / 2 - measureUp::drive::frontLeftLonInset,
                                       -measureUp::chassis::width / 2 + measureUp::drive::frontLeftLatInset};
  frc::Translation2d leverArmFrontRight{measureUp::chassis::length / 2 - measureUp::drive::frontRightLonInset,
                                        measureUp::chassis::width / 2 - measureUp::drive::frontRightLatInset};
  frc::Translation2d leverArmRearRight{-measureUp::chassis::length / 2 + measureUp::drive::rearRightLonInset,
                                       measureUp::chassis::width / 2 - measureUp::drive::rearRightLatInset};
  frc::Translation2d leverArmRearLeft{-measureUp::chassis::length / 2 + measureUp::drive::rearLeftLonInset,
                                      -measureUp::chassis::width / 2 + measureUp::drive::rearLeftLatInset};
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(
      leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

  // Determine max rotate speed
  auto minLeverArm = std::min({leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft},
                              [](const auto& a, const auto& b) { return a.Norm() < b.Norm(); });
  units::length::meter_t turnCircumference = minLeverArm.Norm() * 2 * M_PI;

  m_maxAngularRate = units::degree_t(360.0) * (speedLimits::drive::maxVelocity / turnCircumference);

  // Config Sensors
  CanCoderConfig<sensorConfig::drive::frontLeftTurn>(m_encoderTurnFrontLeft, 100_ms);
  CanCoderConfig<sensorConfig::drive::frontRightTurn>(m_encoderTurnFrontRight, 100_ms);
  CanCoderConfig<sensorConfig::drive::rearRightTurn>(m_encoderTurnRearRight, 100_ms);
  CanCoderConfig<sensorConfig::drive::rearLeftTurn>(m_encoderTurnRearLeft, 100_ms);

  // Configure motors
  FalconConfig<motorConfig::drive::frontLeftDrive>(m_motorDriveFrontLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontRightDrive>(m_motorDriveFrontRight, 100_ms);
  FalconConfig<motorConfig::drive::rearRightDrive>(m_motorDriveRearRight, 100_ms);
  FalconConfig<motorConfig::drive::rearLeftDrive>(m_motorDriveRearLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontLeftTurn>(m_motorTurnFrontLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontRightTurn>(m_motorTurnFrontRight, 100_ms);
  FalconConfig<motorConfig::drive::rearRightTurn>(m_motorTurnRearRight, 100_ms);
  FalconConfig<motorConfig::drive::rearLeftTurn>(m_motorTurnRearLeft, 100_ms);

  InitializeTurnEncoderAngles();

  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};

  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kP, controlLoop::drive::rotate::kP);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kI, controlLoop::drive::rotate::kI);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kD, controlLoop::drive::rotate::kD);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kF, controlLoop::drive::rotate::kF);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::iZone, controlLoop::drive::rotate::iZone);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::allowableError, controlLoop::drive::rotate::allowableError);

  auto tuningSubtable = ntTable->GetSubTable(ntKeys::subsystemDrive::tuning::turn::subtableName);
  tuningSubtable->AddEntryListener([this](NetworkTable* table,
                                          wpi::StringRef key,
                                          nt::NetworkTableEntry entry,
                                          std::shared_ptr<nt::Value> value,
                                          int flags) { this->NTUpdate(table, key, entry, value, flags); },
                                   NT_NOTIFY_UPDATE);
}

void DriveSubsystem::SwerveDrive(const double fwVelocity, const double latVelocity, const double rotateVelocity) {
  // Halt motion
  if (fwVelocity == 0 && latVelocity == 0 && rotateVelocity == 0) {
    m_motorDriveFrontLeft.Set(0.0);
    m_motorTurnFrontLeft.Set(0.0);
    m_motorDriveFrontRight.Set(0.0);
    m_motorTurnFrontRight.Set(0.0);
    m_motorDriveRearRight.Set(0.0);
    m_motorTurnRearRight.Set(0.0);
    m_motorDriveRearLeft.Set(0.0);
    m_motorTurnRearLeft.Set(0.0);
    return;
  }

  auto moduleStates = RawModuleStates(fwVelocity, latVelocity, rotateVelocity);

  // Write desired angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetAngle",
                                 moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetAngle",
                                 moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetAngle",
                                 moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetAngle",
                                 moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetVel",
                                 moduleStates.at(ModuleIndex::frontLeft).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetVel",
                                 moduleStates.at(ModuleIndex::frontRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetVel",
                                 moduleStates.at(ModuleIndex::rearRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetVel", moduleStates.at(ModuleIndex::rearLeft).speed.to<double>());

  std::for_each(
      moduleStates.begin(), moduleStates.end(), [](frc::SwerveModuleState& state) { state.angle = -state.angle; });

  moduleStates.at(ModuleIndex::frontLeft) =
      Optimize(moduleStates.at(ModuleIndex::frontLeft),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontLeft.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontLeft.GetSelectedSensorVelocity()),
               speedLimits::drive::maxVelocity);
  moduleStates.at(ModuleIndex::frontRight) =
      Optimize(moduleStates.at(ModuleIndex::frontRight),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontRight.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontRight.GetSelectedSensorVelocity()),
               speedLimits::drive::maxVelocity);
  moduleStates.at(ModuleIndex::rearRight) =
      Optimize(moduleStates.at(ModuleIndex::rearRight),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearRight.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearRight.GetSelectedSensorVelocity()),
               speedLimits::drive::maxVelocity);
  moduleStates.at(ModuleIndex::rearLeft) =
      Optimize(moduleStates.at(ModuleIndex::rearLeft),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearLeft.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearLeft.GetSelectedSensorVelocity()),
               speedLimits::drive::maxVelocity);

  // Write optimized angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/optimizedTargetAngle",
                                 moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/optimizedTargetAngle",
                                 moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/optimizedTargetAngle",
                                 moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/optimizedTargetAngle",
                                 moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontLeft/optimizedTargetVel",
                                 moduleStates.at(ModuleIndex::frontLeft).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/optimizedTargetVel",
                                 moduleStates.at(ModuleIndex::frontRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/optimizedTargetVel",
                                 moduleStates.at(ModuleIndex::rearRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/optimizedTargetVel",
                                 moduleStates.at(ModuleIndex::rearLeft).speed.to<double>());
  // Write current angles to dashboard
  frc::SmartDashboard::PutNumber(
      "drive/frontLeft/currentAngle",
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition())
          .to<double>());
  frc::SmartDashboard::PutNumber(
      "drive/frontRight/currentAngle",
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition())
          .to<double>());
  frc::SmartDashboard::PutNumber(
      "drive/rearRight/currentAngle",
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition())
          .to<double>());
  frc::SmartDashboard::PutNumber(
      "drive/rearLeft/currentAngle",
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()).to<double>());

  ctre::phoenix::motorcontrol::Faults turnFrontLeftFaults, turnFrontRightFaults, turnRearRightFaults,
      turnRearLeftFaults;
  m_motorTurnFrontLeft.GetFaults(turnFrontLeftFaults);
  m_motorTurnFrontRight.GetFaults(turnFrontRightFaults);
  m_motorTurnRearRight.GetFaults(turnRearRightFaults);
  m_motorTurnRearLeft.GetFaults(turnRearLeftFaults);

  m_motorDriveFrontLeft.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::frontLeft).speed, speedLimits::drive::maxVelocity, turnFrontLeftFaults));
  m_motorTurnFrontLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontLeft).angle.Degrees()));
  m_motorDriveFrontRight.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::frontRight).speed, speedLimits::drive::maxVelocity, turnFrontRightFaults));
  m_motorTurnFrontRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontRight).angle.Degrees()));
  m_motorDriveRearRight.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::rearRight).speed, speedLimits::drive::maxVelocity, turnRearRightFaults));
  m_motorTurnRearRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearRight).angle.Degrees()));
  m_motorDriveRearLeft.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::rearLeft).speed, speedLimits::drive::maxVelocity, turnRearLeftFaults));
  m_motorTurnRearLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearLeft).angle.Degrees()));
}

void DriveSubsystem::Home(const units::degree_t currentAngle) {
  const argos_lib::swerve::SwerveModulePositions newHomes{
      argos_lib::swerve::ConstrainAngle(
          units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) - currentAngle,
          0_deg,
          360_deg),
      argos_lib::swerve::ConstrainAngle(
          units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) - currentAngle,
          0_deg,
          360_deg),
      argos_lib::swerve::ConstrainAngle(
          units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) - currentAngle,
          0_deg,
          360_deg),
      argos_lib::swerve::ConstrainAngle(
          units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) - currentAngle,
          0_deg,
          360_deg)};

  m_pHomingStorage->Save(newHomes);

  // SetPosition expects a value in degrees
  m_encoderTurnFrontLeft.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(currentAngle.to<double>(), 50);
}

void DriveSubsystem::SetFieldOrientation(const units::degree_t currentAngle) {
  /// @todo Apply offset instead of always set to 0
  m_fieldOrientationOffset = m_IMU.GetRotation2d();
}

void DriveSubsystem::SetControlMode(const ControlMode newControlMode) {
  switch (newControlMode) {
    case ControlMode::fieldCentric:
      std::printf("Switching to field-centric control\n");
      break;
    case ControlMode::robotCentric:
      std::printf("Switching to robot-centric control\n");
      break;
  }
  m_activeControlMode = newControlMode;
}

void DriveSubsystem::InitializeTurnEncoderAngles() {
  const auto loadedHomes = m_pHomingStorage->Load();
  if (loadedHomes) {
    const auto curFrontLeftPosition =
        units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) - loadedHomes.value().FrontLeft;
    const auto curFrontRighPosition = units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) -
                                      loadedHomes.value().FrontRight;
    const auto curRearRightPosition =
        units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) - loadedHomes.value().RearRight;
    const auto curRearLeftPosition =
        units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) - loadedHomes.value().RearLeft;

    // SetPosition expects a value in degrees
    m_encoderTurnFrontLeft.SetPosition(curFrontLeftPosition.to<double>(), 50);
    m_encoderTurnFrontRight.SetPosition(curFrontRighPosition.to<double>(), 50);
    m_encoderTurnRearRight.SetPosition(curRearRightPosition.to<double>(), 50);
    m_encoderTurnRearLeft.SetPosition(curRearLeftPosition.to<double>(), 50);
  }
}

void DriveSubsystem::NTUpdate(
    NetworkTable* table, wpi::StringRef key, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags) {
  if (entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kP) {
    m_motorTurnFrontLeft.Config_kP(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kP(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kP(0, value->GetDouble());
    m_motorTurnRearRight.Config_kP(0, value->GetDouble());
  } else if (entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kI) {
    m_motorTurnFrontLeft.Config_kI(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kI(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kI(0, value->GetDouble());
    m_motorTurnRearRight.Config_kI(0, value->GetDouble());
  } else if (entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kD) {
    m_motorTurnFrontLeft.Config_kD(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kD(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kD(0, value->GetDouble());
    m_motorTurnRearRight.Config_kD(0, value->GetDouble());
  } else if (entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kF) {
    m_motorTurnFrontLeft.Config_kF(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kF(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kF(0, value->GetDouble());
    m_motorTurnRearRight.Config_kF(0, value->GetDouble());
  } else if (entry.GetName() ==
             std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::iZone) {
    m_motorTurnFrontLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnFrontRight.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearRight.Config_IntegralZone(0, value->GetDouble());
  } else if (entry.GetName() ==
             std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::allowableError) {
    m_motorTurnFrontLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnFrontRight.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearRight.ConfigAllowableClosedloopError(0, value->GetDouble());
  }
}

double DriveSubsystem::ModuleDriveSpeed(const units::velocity::feet_per_second_t desiredSpeed,
                                        const units::velocity::feet_per_second_t maxSpeed,
                                        const ctre::phoenix::motorcontrol::Faults turnFaults) {
  const bool fatalFault = turnFaults.RemoteLossOfSignal || turnFaults.HardwareFailure || turnFaults.APIError;
  return fatalFault ? 0.0 : (desiredSpeed / maxSpeed).to<double>();
}

wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::RawModuleStates(const double fwVelocity,
                                                                      const double latVelocity,
                                                                      const double rotateVelocity) {
  const auto desiredFwVelocity = speedLimits::drive::maxVelocity * fwVelocity;
  const auto desiredLatVelocity = speedLimits::drive::maxVelocity * latVelocity;
  const auto desiredRotVelocity = m_maxAngularRate * rotateVelocity;
  switch (m_activeControlMode) {
    case ControlMode::fieldCentric: {
      const auto rotationToApply = m_IMU.GetRotation2d() - m_fieldOrientationOffset;
      return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          desiredFwVelocity, desiredLatVelocity, desiredRotVelocity, -rotationToApply));
    }
    case ControlMode::robotCentric:
      return m_pSwerveKinematicsModel->ToSwerveModuleStates(
          frc::ChassisSpeeds{desiredFwVelocity, desiredLatVelocity, desiredRotVelocity});
  }
  // This shouldn't be reachable (and there will be a compiler warning if a switch case is unhandled), but stop if in unknown drive state
  return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds{0_mps, 0_mps, 0_rpm});
}
