#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include "general/swerveUtils.h"
#include "subsystems/DriveSubsystem.h"
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
  m_encoderTurnFrontLeft.ConfigFactoryDefault(100);
  m_encoderTurnFrontLeft.ConfigSensorDirection(sensorConfig::drive::frontLeftTurn::direction, 100);
  m_encoderTurnFrontLeft.ConfigSensorInitializationStrategy(sensorConfig::drive::frontLeftTurn::initMode, 100);
  m_encoderTurnFrontLeft.ConfigAbsoluteSensorRange(sensorConfig::drive::frontLeftTurn::range, 100);
  m_encoderTurnFrontLeft.ConfigMagnetOffset(sensorConfig::drive::frontLeftTurn::magOffset, 100);

  m_encoderTurnFrontRight.ConfigFactoryDefault(100);
  m_encoderTurnFrontRight.ConfigSensorDirection(sensorConfig::drive::frontRightTurn::direction, 100);
  m_encoderTurnFrontRight.ConfigSensorInitializationStrategy(sensorConfig::drive::frontRightTurn::initMode, 100);
  m_encoderTurnFrontRight.ConfigAbsoluteSensorRange(sensorConfig::drive::frontRightTurn::range, 100);
  m_encoderTurnFrontRight.ConfigMagnetOffset(sensorConfig::drive::frontRightTurn::magOffset, 100);

  m_encoderTurnRearRight.ConfigFactoryDefault(100);
  m_encoderTurnRearRight.ConfigSensorDirection(sensorConfig::drive::rearRightTurn::direction, 100);
  m_encoderTurnRearRight.ConfigSensorInitializationStrategy(sensorConfig::drive::rearRightTurn::initMode, 100);
  m_encoderTurnRearRight.ConfigAbsoluteSensorRange(sensorConfig::drive::rearRightTurn::range, 100);
  m_encoderTurnRearRight.ConfigMagnetOffset(sensorConfig::drive::rearRightTurn::magOffset, 100);

  m_encoderTurnRearLeft.ConfigFactoryDefault(100);
  m_encoderTurnRearLeft.ConfigSensorDirection(sensorConfig::drive::rearLeftTurn::direction, 100);
  m_encoderTurnRearLeft.ConfigSensorInitializationStrategy(sensorConfig::drive::rearLeftTurn::initMode, 100);
  m_encoderTurnRearLeft.ConfigAbsoluteSensorRange(sensorConfig::drive::rearLeftTurn::range, 100);
  m_encoderTurnRearLeft.ConfigMagnetOffset(sensorConfig::drive::rearLeftTurn::magOffset, 100);

  // Configure motors
  m_motorDriveFrontLeft.ConfigFactoryDefault(100);
  m_motorDriveFrontLeft.SetInverted(motorConfig::drive::frontLeftDrive::inverted);
  m_motorDriveFrontLeft.SetSensorPhase(motorConfig::drive::frontLeftDrive::sensorPhase);
  m_motorDriveFrontLeft.SetNeutralMode(motorConfig::drive::frontLeftDrive::neutralMode);
  m_motorDriveFrontLeft.ConfigVoltageCompSaturation(motorConfig::drive::frontLeftDrive::voltCompSat, 100);

  m_motorDriveFrontRight.ConfigFactoryDefault(100);
  m_motorDriveFrontRight.SetInverted(motorConfig::drive::frontRightDrive::inverted);
  m_motorDriveFrontRight.SetSensorPhase(motorConfig::drive::frontRightDrive::sensorPhase);
  m_motorDriveFrontRight.SetNeutralMode(motorConfig::drive::frontRightDrive::neutralMode);
  m_motorDriveFrontRight.ConfigVoltageCompSaturation(motorConfig::drive::frontRightDrive::voltCompSat, 100);

  m_motorDriveRearRight.ConfigFactoryDefault(100);
  m_motorDriveRearRight.SetInverted(motorConfig::drive::rearRightDrive::inverted);
  m_motorDriveRearRight.SetSensorPhase(motorConfig::drive::rearRightDrive::sensorPhase);
  m_motorDriveRearRight.SetNeutralMode(motorConfig::drive::rearRightDrive::neutralMode);
  m_motorDriveRearRight.ConfigVoltageCompSaturation(motorConfig::drive::rearRightDrive::voltCompSat, 100);

  m_motorDriveRearLeft.ConfigFactoryDefault(100);
  m_motorDriveRearLeft.SetInverted(motorConfig::drive::rearLeftDrive::inverted);
  m_motorDriveRearLeft.SetSensorPhase(motorConfig::drive::rearLeftDrive::sensorPhase);
  m_motorDriveRearLeft.SetNeutralMode(motorConfig::drive::rearLeftDrive::neutralMode);
  m_motorDriveRearLeft.ConfigVoltageCompSaturation(motorConfig::drive::rearLeftDrive::voltCompSat, 100);

  m_motorTurnFrontLeft.ConfigFactoryDefault(100);
  m_motorTurnFrontLeft.SetInverted(motorConfig::drive::frontLeftTurn::inverted);
  m_motorTurnFrontLeft.SetSensorPhase(motorConfig::drive::frontLeftTurn::sensorPhase);
  m_motorTurnFrontLeft.SetNeutralMode(motorConfig::drive::frontLeftTurn::neutralMode);
  m_motorTurnFrontLeft.ConfigVoltageCompSaturation(motorConfig::drive::frontLeftTurn::voltCompSat, 100);

  m_motorTurnFrontRight.ConfigFactoryDefault(100);
  m_motorTurnFrontRight.SetInverted(motorConfig::drive::frontRightTurn::inverted);
  m_motorTurnFrontRight.SetSensorPhase(motorConfig::drive::frontRightTurn::sensorPhase);
  m_motorTurnFrontRight.SetNeutralMode(motorConfig::drive::frontRightTurn::neutralMode);
  m_motorTurnFrontRight.ConfigVoltageCompSaturation(motorConfig::drive::frontRightTurn::voltCompSat, 100);

  m_motorTurnRearRight.ConfigFactoryDefault(100);
  m_motorTurnRearRight.SetInverted(motorConfig::drive::rearRightTurn::inverted);
  m_motorTurnRearRight.SetSensorPhase(motorConfig::drive::rearRightTurn::sensorPhase);
  m_motorTurnRearRight.SetNeutralMode(motorConfig::drive::rearRightTurn::neutralMode);
  m_motorTurnRearRight.ConfigVoltageCompSaturation(motorConfig::drive::rearRightTurn::voltCompSat, 100);

  m_motorTurnRearLeft.ConfigFactoryDefault(100);
  m_motorTurnRearLeft.SetInverted(motorConfig::drive::rearLeftTurn::inverted);
  m_motorTurnRearLeft.SetSensorPhase(motorConfig::drive::rearLeftTurn::sensorPhase);
  m_motorTurnRearLeft.SetNeutralMode(motorConfig::drive::rearLeftTurn::neutralMode);
  m_motorTurnRearLeft.ConfigVoltageCompSaturation(motorConfig::drive::rearLeftTurn::voltCompSat, 100);

  // Assign feedback sensors to motors
  m_motorTurnFrontLeft.ConfigRemoteFeedbackFilter(m_encoderTurnFrontLeft.GetDeviceNumber(),
                                                  ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder,
                                                  0, 100);
  m_motorTurnFrontLeft.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0, 0, 100);
  m_motorTurnFrontLeft.Config_kP(0, controlLoop::drive::rotate::kP, 100);
  m_motorTurnFrontLeft.Config_kI(0, controlLoop::drive::rotate::kI, 100);
  m_motorTurnFrontLeft.Config_kD(0, controlLoop::drive::rotate::kD, 100);
  m_motorTurnFrontLeft.Config_kF(0, controlLoop::drive::rotate::kF, 100);
  m_motorTurnFrontLeft.Config_IntegralZone(0, controlLoop::drive::rotate::iZone, 100);
  m_motorTurnFrontLeft.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError, 100);

  m_motorTurnFrontRight.ConfigRemoteFeedbackFilter(m_encoderTurnFrontRight.GetDeviceNumber(),
                                                   ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder,
                                                   0, 100);
  m_motorTurnFrontRight.ConfigRemoteFeedbackFilter(address::encoder::frontRightTurn, ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
  m_motorTurnFrontRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0, 0, 100);
  m_motorTurnFrontRight.Config_kP(0, controlLoop::drive::rotate::kP, 100);
  m_motorTurnFrontRight.Config_kI(0, controlLoop::drive::rotate::kI, 100);
  m_motorTurnFrontRight.Config_kD(0, controlLoop::drive::rotate::kD, 100);
  m_motorTurnFrontRight.Config_kF(0, controlLoop::drive::rotate::kF, 100);
  m_motorTurnFrontRight.Config_IntegralZone(0, controlLoop::drive::rotate::iZone, 100);
  m_motorTurnFrontRight.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError, 100);

  m_motorTurnRearLeft.ConfigRemoteFeedbackFilter(m_encoderTurnRearLeft.GetDeviceNumber(),
                                                 ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder,
                                                 0, 100);
  m_motorTurnRearLeft.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0, 0, 100);
  m_motorTurnRearLeft.Config_kP(0, controlLoop::drive::rotate::kP, 100);
  m_motorTurnRearLeft.Config_kI(0, controlLoop::drive::rotate::kI, 100);
  m_motorTurnRearLeft.Config_kD(0, controlLoop::drive::rotate::kD, 100);
  m_motorTurnRearLeft.Config_kF(0, controlLoop::drive::rotate::kF, 100);
  m_motorTurnRearLeft.Config_IntegralZone(0, controlLoop::drive::rotate::iZone, 100);
  m_motorTurnRearLeft.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError, 100);

  m_motorTurnRearRight.ConfigRemoteFeedbackFilter(m_encoderTurnRearRight.GetDeviceNumber(),
                                                  ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder,
                                                  0, 100);
  m_motorTurnRearRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0, 0, 100);
  m_motorTurnRearRight.Config_kP(0, controlLoop::drive::rotate::kP, 100);
  m_motorTurnRearRight.Config_kI(0, controlLoop::drive::rotate::kI, 100);
  m_motorTurnRearRight.Config_kD(0, controlLoop::drive::rotate::kD, 100);
  m_motorTurnRearRight.Config_kF(0, controlLoop::drive::rotate::kF, 100);
  m_motorTurnRearRight.Config_IntegralZone(0, controlLoop::drive::rotate::iZone, 100);
  m_motorTurnRearRight.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError, 100);

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
  // Write current angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()).to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/currentAngle", measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()).to<double>());

  m_motorDriveFrontLeft.Set(moduleStates.at(ModuleIndex::frontLeft).speed / speedLimits::drive::maxVelocity);
  m_motorTurnFrontLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontLeft).angle.Degrees()));
  m_motorDriveFrontRight.Set(moduleStates.at(ModuleIndex::frontRight).speed / speedLimits::drive::maxVelocity);
  m_motorTurnFrontRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontRight).angle.Degrees()));
  m_motorDriveRearRight.Set(moduleStates.at(ModuleIndex::rearRight).speed / speedLimits::drive::maxVelocity);
  m_motorTurnRearRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearRight).angle.Degrees()));
  m_motorDriveRearLeft.Set(moduleStates.at(ModuleIndex::rearLeft).speed / speedLimits::drive::maxVelocity);
  m_motorTurnRearLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearLeft).angle.Degrees()));
}

void DriveSubsystem::Home(const units::degree_t currentAngle) {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};

  m_encoderTurnFrontLeft.SetPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle), 10);
  m_encoderTurnFrontRight.SetPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle), 10);
  m_encoderTurnRearRight.SetPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle), 10);
  m_encoderTurnRearLeft.SetPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle), 10);

  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, m_encoderTurnFrontLeft.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, m_encoderTurnFrontRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, m_encoderTurnRearRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, m_encoderTurnRearLeft.GetAbsolutePosition());
}

void DriveSubsystem::InitializeTurnEncoderAngles() {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  auto homePositionTurnFrontLeft = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, 0);
  auto homePositionTurnFrontRight = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, 0);
  auto homePositionTurnRearRight = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, 0);
  auto homePositionTurnRearLeft = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, 0);

  auto curFrontLeftPosition = units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition() - homePositionTurnFrontLeft);
  auto curFrontRighPosition = units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition() - homePositionTurnFrontRight);
  auto curRearRightPosition = units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition() - homePositionTurnRearRight);
  auto curRearLeftPosition = units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition() - homePositionTurnRearLeft);

  m_encoderTurnFrontLeft.SetPosition(curFrontLeftPosition.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(curFrontRighPosition.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(curRearRightPosition.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(curRearLeftPosition.to<double>(), 50);
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
