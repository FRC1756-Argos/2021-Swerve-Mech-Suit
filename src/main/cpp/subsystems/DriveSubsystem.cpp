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
  frc::Translation2d leverArmFrontLeft {-measureUp::chassis::width/2 + measureUp::drive::frontLeftLatInset,
                                         measureUp::chassis::length/2 - measureUp::drive::frontLeftLonInset};
  frc::Translation2d leverArmFrontRight{ measureUp::chassis::width/2 - measureUp::drive::frontRightLatInset,
                                         measureUp::chassis::length/2 - measureUp::drive::frontRightLonInset};
  frc::Translation2d leverArmRearRight{  measureUp::chassis::width/2 - measureUp::drive::rearRightLatInset,
                                        -measureUp::chassis::length/2 + measureUp::drive::rearRightLonInset};
  frc::Translation2d leverArmRearLeft{  -measureUp::chassis::width/2 + measureUp::drive::rearLeftLatInset,
                                        -measureUp::chassis::length/2 + measureUp::drive::rearLeftLonInset};
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

  auto minLeverArm = std::min({leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft},
                              [](const auto& a, const auto& b) { return a.Norm() < b.Norm(); });
  units::length::meter_t turnCircumference = minLeverArm.Norm() * 2 * M_PI;

  m_maxAngularRate = units::degree_t(360.0) * (speedLimits::drive::maxVelocity / turnCircumference);
}

void DriveSubsystem::SwerveDrive(const double fwVelocity,
                                 const double latVelocity,
                                 const double rotateVelocity) {
  auto moduleStates = m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds{ speedLimits::drive::maxVelocity * fwVelocity,
                                                                                         speedLimits::drive::maxVelocity * latVelocity,
                                                                                         m_maxAngularRate * rotateVelocity });
  m_motorDriveFrontLeft.Set(moduleStates.at(ModuleIndex::frontLeft).speed / speedLimits::drive::maxVelocity);
  m_motorTurnFrontLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  m_motorDriveFrontRight.Set(moduleStates.at(ModuleIndex::frontRight).speed / speedLimits::drive::maxVelocity);
  m_motorTurnFrontRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  m_motorDriveRearRight.Set(moduleStates.at(ModuleIndex::rearRight).speed / speedLimits::drive::maxVelocity);
  m_motorTurnRearRight.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  m_motorDriveRearLeft.Set(moduleStates.at(ModuleIndex::rearLeft).speed / speedLimits::drive::maxVelocity);
  m_motorTurnRearLeft.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
