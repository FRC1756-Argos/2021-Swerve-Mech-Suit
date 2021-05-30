#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

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
  frc::Translation2d leverArmFrontLeft {-measureUp::chassis::width/2 + measureUp::drive::frontLeftLatInset,
                                         measureUp::chassis::length/2 - measureUp::drive::frontLeftLonInset};
  frc::Translation2d leverArmFrontRight{ measureUp::chassis::width/2 - measureUp::drive::frontRightLatInset,
                                         measureUp::chassis::length/2 - measureUp::drive::frontRightLonInset};
  frc::Translation2d leverArmRearRight{  measureUp::chassis::width/2 - measureUp::drive::rearRightLatInset,
                                        -measureUp::chassis::length/2 + measureUp::drive::rearRightLonInset};
  frc::Translation2d leverArmRearLeft{  -measureUp::chassis::width/2 + measureUp::drive::rearLeftLatInset,
                                        -measureUp::chassis::length/2 + measureUp::drive::rearLeftLonInset};
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

  // Determine max rotate speed
  auto minLeverArm = std::min({leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft},
                              [](const auto& a, const auto& b) { return a.Norm() < b.Norm(); });
  units::length::meter_t turnCircumference = minLeverArm.Norm() * 2 * M_PI;

  m_maxAngularRate = units::degree_t(360.0) * (speedLimits::drive::maxVelocity / turnCircumference);

  // Assign feedback sensors to motors
  m_motorTurnFrontLeft.ConfigRemoteFeedbackFilter(address::encoder::frontLeftTurn, ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
  m_motorTurnFrontLeft.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0);
  m_motorTurnFrontLeft.Config_kP(0, controlLoop::drive::rotate::kP);
  m_motorTurnFrontLeft.Config_kI(0, controlLoop::drive::rotate::kI);
  m_motorTurnFrontLeft.Config_kD(0, controlLoop::drive::rotate::kD);
  m_motorTurnFrontLeft.Config_kF(0, controlLoop::drive::rotate::kF);
  m_motorTurnFrontLeft.Config_IntegralZone(0, controlLoop::drive::rotate::iZone);
  m_motorTurnFrontLeft.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError);

  m_motorTurnFrontRight.ConfigRemoteFeedbackFilter(address::encoder::frontRightTurn, ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
  m_motorTurnFrontRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0);
  m_motorTurnFrontRight.Config_kP(0, controlLoop::drive::rotate::kP);
  m_motorTurnFrontRight.Config_kI(0, controlLoop::drive::rotate::kI);
  m_motorTurnFrontRight.Config_kD(0, controlLoop::drive::rotate::kD);
  m_motorTurnFrontRight.Config_kF(0, controlLoop::drive::rotate::kF);
  m_motorTurnFrontRight.Config_IntegralZone(0, controlLoop::drive::rotate::iZone);
  m_motorTurnFrontRight.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError);

  m_motorTurnRearLeft.ConfigRemoteFeedbackFilter(address::encoder::rearLeftTurn, ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
  m_motorTurnRearLeft.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0);
  m_motorTurnRearLeft.Config_kP(0, controlLoop::drive::rotate::kP);
  m_motorTurnRearLeft.Config_kI(0, controlLoop::drive::rotate::kI);
  m_motorTurnRearLeft.Config_kD(0, controlLoop::drive::rotate::kD);
  m_motorTurnRearLeft.Config_kF(0, controlLoop::drive::rotate::kF);
  m_motorTurnRearLeft.Config_IntegralZone(0, controlLoop::drive::rotate::iZone);
  m_motorTurnRearLeft.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError);

  m_motorTurnRearRight.ConfigRemoteFeedbackFilter(address::encoder::rearRightTurn, ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
  m_motorTurnRearRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0);
  m_motorTurnRearRight.Config_kP(0, controlLoop::drive::rotate::kP);
  m_motorTurnRearRight.Config_kI(0, controlLoop::drive::rotate::kI);
  m_motorTurnRearRight.Config_kD(0, controlLoop::drive::rotate::kD);
  m_motorTurnRearRight.Config_kF(0, controlLoop::drive::rotate::kF);
  m_motorTurnRearRight.Config_IntegralZone(0, controlLoop::drive::rotate::iZone);
  m_motorTurnRearRight.ConfigAllowableClosedloopError(0, controlLoop::drive::rotate::allowableError);

  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  auto homePositionTurnFrontLeft = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, 0);
  auto homePositionTurnFrontRight = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, 0);
  auto homePositionTurnRearRight = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, 0);
  auto homePositionTurnRearLeft = ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, 0);

  // Set initial positions
  auto constrainAngle = [](double inAngle){
    inAngle = fmod(inAngle,360);
    if (inAngle < 0) {
        inAngle += 360;
    }
    return inAngle;
  };

  m_motorTurnFrontLeft.SetSelectedSensorPosition(constrainAngle(m_encoderTurnFrontLeft.GetAbsolutePosition() - homePositionTurnFrontLeft));
  m_motorTurnFrontRight.SetSelectedSensorPosition(constrainAngle(m_encoderTurnFrontRight.GetAbsolutePosition() - homePositionTurnFrontRight));
  m_motorTurnRearLeft.SetSelectedSensorPosition(constrainAngle(m_encoderTurnRearRight.GetAbsolutePosition() - homePositionTurnRearRight));
  m_motorTurnRearRight.SetSelectedSensorPosition(constrainAngle(m_encoderTurnRearLeft.GetAbsolutePosition() - homePositionTurnRearLeft));

  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kP, controlLoop::drive::rotate::kP);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kI, controlLoop::drive::rotate::kI);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kD, controlLoop::drive::rotate::kD);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::kF, controlLoop::drive::rotate::kF);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::iZone, controlLoop::drive::rotate::iZone);
  ntTable->PutNumber(ntKeys::subsystemDrive::tuning::turn::allowableError, controlLoop::drive::rotate::allowableError);

  ntTable->AddEntryListener([this](NetworkTable* table, wpi::StringRef key, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags)
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

void DriveSubsystem::Home(const units::degree_t currentAngle){
  m_motorTurnFrontLeft.SetSelectedSensorPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle));
  m_motorTurnFrontRight.SetSelectedSensorPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle));
  m_motorTurnRearRight.SetSelectedSensorPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle));
  m_motorTurnRearLeft.SetSelectedSensorPosition(measureUp::sensorConversion::swerveRotate::fromAngle(currentAngle));

  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, m_encoderTurnFrontLeft.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, m_encoderTurnFrontRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, m_encoderTurnRearRight.GetAbsolutePosition());
  ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, m_encoderTurnRearLeft.GetAbsolutePosition());
}

void DriveSubsystem::NTUpdate(NetworkTable* table,
                              wpi::StringRef key,
                              nt::NetworkTableEntry entry,
                              std::shared_ptr<nt::Value> value,
                              int flags) {
  if(key == ntKeys::subsystemDrive::tuning::turn::kP) {
    m_motorTurnFrontLeft.Config_kP(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kP(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kP(0, value->GetDouble());
    m_motorTurnRearRight.Config_kP(0, value->GetDouble());
  }
  else if(key == ntKeys::subsystemDrive::tuning::turn::kI) {
    m_motorTurnFrontLeft.Config_kI(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kI(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kI(0, value->GetDouble());
    m_motorTurnRearRight.Config_kI(0, value->GetDouble());
  }
  else if(key == ntKeys::subsystemDrive::tuning::turn::kD) {
    m_motorTurnFrontLeft.Config_kD(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kD(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kD(0, value->GetDouble());
    m_motorTurnRearRight.Config_kD(0, value->GetDouble());
  }
  else if(key == ntKeys::subsystemDrive::tuning::turn::kF) {
    m_motorTurnFrontLeft.Config_kF(0, value->GetDouble());
    m_motorTurnFrontRight.Config_kF(0, value->GetDouble());
    m_motorTurnRearLeft.Config_kF(0, value->GetDouble());
    m_motorTurnRearRight.Config_kF(0, value->GetDouble());
  }
  else if(key == ntKeys::subsystemDrive::tuning::turn::iZone) {
    m_motorTurnFrontLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnFrontRight.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearLeft.Config_IntegralZone(0, value->GetDouble());
    m_motorTurnRearRight.Config_IntegralZone(0, value->GetDouble());
  }
  else if(key == ntKeys::subsystemDrive::tuning::turn::allowableError) {
    m_motorTurnFrontLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnFrontRight.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearLeft.ConfigAllowableClosedloopError(0, value->GetDouble());
    m_motorTurnRearRight.ConfigAllowableClosedloopError(0, value->GetDouble());
  }
}
