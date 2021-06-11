#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include "general/swerveUtils.h"
#include "subsystems/DriveSubsystem.h"
#include "config/falconConfig.h"
#include "config/canCoderConfig.h"
#include "Constants.h"

DriveSubsystem::DriveSubsystem() : m_motorDriveFrontLeft(address::motor::frontLeftDrive),
                                   m_motorDriveFrontRight(address::motor::frontRightDrive),
                                   m_motorDriveRearRight(address::motor::rearRightDrive),
                                   m_motorDriveRearLeft(address::motor::rearLeftDrive),
                                   m_motorTurnFrontLeft(address::motor::frontLeftTurn),
                                   m_motorTurnFrontRight(address::motor::frontRightTurn),
                                   m_motorTurnRearRight(address::motor::rearRightTurn),
                                   m_motorTurnRearLeft(address::motor::rearLeftTurn),
                                   m_encoderTurnFrontLeft(address::encoder::frontLeftTurn),
                                   m_encoderTurnFrontRight(address::encoder::frontRightTurn),
                                   m_encoderTurnRearRight(address::encoder::rearRightTurn),
                                   m_encoderTurnRearLeft(address::encoder::rearLeftTurn)
{
  // Set lever arms for each module
  frc::Translation2d leverArmFrontLeft {  measureUp::chassis::length/2 - measureUp::drive::frontLeftLonInset,
                                         -measureUp::chassis::width/2  + measureUp::drive::frontLeftLatInset };
  frc::Translation2d leverArmFrontRight{ measureUp::chassis::length/2 - measureUp::drive::frontRightLonInset,
                                         measureUp::chassis::width/2  - measureUp::drive::frontRightLatInset };
  frc::Translation2d leverArmRearRight{ -measureUp::chassis::length/2 + measureUp::drive::rearRightLonInset,
                                         measureUp::chassis::width/2  - measureUp::drive::rearRightLatInset };
  frc::Translation2d leverArmRearLeft{ -measureUp::chassis::length/2 + measureUp::drive::rearLeftLonInset,
                                       -measureUp::chassis::width/2  + measureUp::drive::rearLeftLatInset };
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

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
  tuningSubtable->AddEntryListener([this](NetworkTable* table, wpi::StringRef key, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags)
                                     {this->NTUpdate(table, key, entry, value, flags);},
                                   NT_NOTIFY_UPDATE);
}

void DriveSubsystem::SwerveDrive(const double fwVelocity,
                                 const double latVelocity,
                                 const double rotateVelocity) {
  // Halt motion
  if(fwVelocity == 0 && latVelocity == 0 && rotateVelocity == 0) {
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

  auto moduleStates = m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds{ speedLimits::drive::maxVelocity * fwVelocity,
                                                                                         speedLimits::drive::maxVelocity * latVelocity,
                                                                                         m_maxAngularRate * rotateVelocity });

  // Write desired angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetAngle", moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetAngle", moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetAngle", moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetAngle", moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetVel", moduleStates.at(ModuleIndex::frontLeft).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetVel", moduleStates.at(ModuleIndex::frontRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetVel", moduleStates.at(ModuleIndex::rearRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetVel", moduleStates.at(ModuleIndex::rearLeft).speed.to<double>());

  std::for_each(moduleStates.begin(), moduleStates.end(),
                [](frc::SwerveModuleState& state){ state.angle = -state.angle; });

  moduleStates.at(ModuleIndex::frontLeft) = Optimize( moduleStates.at(ModuleIndex::frontLeft),
                                                      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()),
                                                      measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontLeft.GetSelectedSensorVelocity()),
                                                      measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontLeft.GetSelectedSensorVelocity()) );
  moduleStates.at(ModuleIndex::frontRight) = Optimize( moduleStates.at(ModuleIndex::frontRight),
                                                       measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()),
                                                       measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontRight.GetSelectedSensorVelocity()),
                                                       measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontRight.GetSelectedSensorVelocity()) );
  moduleStates.at(ModuleIndex::rearRight) = Optimize( moduleStates.at(ModuleIndex::rearRight),
                                                      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()),
                                                      measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearRight.GetSelectedSensorVelocity()),
                                                      measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearRight.GetSelectedSensorVelocity()) );
  moduleStates.at(ModuleIndex::rearLeft) = Optimize( moduleStates.at(ModuleIndex::rearLeft),
                                                     measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()),
                                                     measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearLeft.GetSelectedSensorVelocity()),
                                                     measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearLeft.GetSelectedSensorVelocity()) );

  // Write optimized angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/optimizedTargetAngle", moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/optimizedTargetAngle", moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/optimizedTargetAngle", moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/optimizedTargetAngle", moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontLeft/optimizedTargetVel", moduleStates.at(ModuleIndex::frontLeft).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/optimizedTargetVel", moduleStates.at(ModuleIndex::frontRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/optimizedTargetVel", moduleStates.at(ModuleIndex::rearRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/optimizedTargetVel", moduleStates.at(ModuleIndex::rearLeft).speed.to<double>());
  // Write current angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()).to<double>());

  ctre::phoenix::motorcontrol::Faults turnFrontLeftFaults, turnFrontRightFaults, turnRearRightFaults, turnRearLeftFaults;
  m_motorTurnFrontLeft.GetFaults(turnFrontLeftFaults);
  m_motorTurnFrontRight.GetFaults(turnFrontRightFaults);
  m_motorTurnRearRight.GetFaults(turnRearRightFaults);
  m_motorTurnRearLeft.GetFaults(turnRearLeftFaults);

  m_motorDriveFrontLeft.Set(ModuleDriveSpeed(moduleStates.at(ModuleIndex::frontLeft).speed,
                                             speedLimits::drive::maxVelocity,
                                             turnFrontLeftFaults));
  m_motorTurnFrontLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontLeft).angle.Degrees()));
  m_motorDriveFrontRight.Set(ModuleDriveSpeed(moduleStates.at(ModuleIndex::frontRight).speed,
                                             speedLimits::drive::maxVelocity,
                                             turnFrontRightFaults));
  m_motorTurnFrontRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontRight).angle.Degrees()));
  m_motorDriveRearRight.Set(ModuleDriveSpeed(moduleStates.at(ModuleIndex::rearRight).speed,
                                             speedLimits::drive::maxVelocity,
                                             turnRearRightFaults));
  m_motorTurnRearRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearRight).angle.Degrees()));
  m_motorDriveRearLeft.Set(ModuleDriveSpeed(moduleStates.at(ModuleIndex::rearLeft).speed,
                                             speedLimits::drive::maxVelocity,
                                             turnRearLeftFaults));
  m_motorTurnRearLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearLeft).angle.Degrees()));
}

void DriveSubsystem::Home(const units::degree_t currentAngle) {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};

  // SetPosition expects a value in degrees
  m_encoderTurnFrontLeft.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(currentAngle.to<double>(), 50);

  // GetAbsolutePosition returns degrees in configured range
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, m_encoderTurnFrontLeft.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, m_encoderTurnFrontRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, m_encoderTurnRearRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, m_encoderTurnRearLeft.GetAbsolutePosition());
}

void DriveSubsystem::InitializeTurnEncoderAngles() {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  // Read positions are in degrees
  const auto homePositionTurnFrontLeft = units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, 0));
  const auto homePositionTurnFrontRight = units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, 0));
  const auto homePositionTurnRearRight = units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, 0));
  const auto homePositionTurnRearLeft = units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, 0));

  const auto curFrontLeftPosition = units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) - homePositionTurnFrontLeft;
  const auto curFrontRighPosition = units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) - homePositionTurnFrontRight;
  const auto curRearRightPosition = units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) - homePositionTurnRearRight;
  const auto curRearLeftPosition = units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) - homePositionTurnRearLeft;

  // SetPosition expects a value in degrees
  m_encoderTurnFrontLeft.SetPosition(curFrontLeftPosition.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(curFrontRighPosition.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(curRearRightPosition.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(curRearLeftPosition.to<double>(), 50);
}

void DriveSubsystem::NTUpdate(NetworkTable* table,
                              wpi::StringRef key,
                              nt::NetworkTableEntry entry,
                              std::shared_ptr<nt::Value> value,
                              int flags) {
  if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kP) {
    m_motorTurnFrontLeft.Config_kP(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kP(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kP(0, value->GetDouble());
    m_motorTurnRearRight.Config_kP(0, value->GetDouble());
  }
  else if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kI) {
    m_motorTurnFrontLeft.Config_kI(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kI(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kI(0, value->GetDouble());
    m_motorTurnRearRight.Config_kI(0, value->GetDouble());
  }
  else if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kD) {
    m_motorTurnFrontLeft.Config_kD(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kD(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kD(0, value->GetDouble());
    m_motorTurnRearRight.Config_kD(0, value->GetDouble());
  }
  else if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::kF) {
    m_motorTurnFrontLeft.Config_kF(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kF(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kF(0, value->GetDouble());
    m_motorTurnRearRight.Config_kF(0, value->GetDouble());
  }
  else if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::iZone) {
    m_motorTurnFrontLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnFrontRight.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearRight.Config_IntegralZone(0, value->GetDouble());
  }
  else if(entry.GetName() == std::string("/") + ntKeys::tableName + "/" + ntKeys::subsystemDrive::tuning::turn::allowableError) {
    m_motorTurnFrontLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnFrontRight.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearRight.ConfigAllowableClosedloopError(0, value->GetDouble());
  }
}

double DriveSubsystem::ModuleDriveSpeed(const units::velocity::feet_per_second_t desiredSpeed,
                                        const units::velocity::feet_per_second_t maxSpeed,
                                        const ctre::phoenix::motorcontrol::Faults turnFaults) {
  const bool fatalFault = turnFaults.RemoteLossOfSignal ||
                          turnFaults.HardwareFailure ||
                          turnFaults.APIError;
  return fatalFault ? 0.0 :
                      (desiredSpeed / maxSpeed).to<double>();
}
