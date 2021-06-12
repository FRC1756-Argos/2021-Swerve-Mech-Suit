#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include "adi/ADIS16448_IMU.h"
#include "ctre/Phoenix.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  enum ModuleIndex {
    frontLeft,
    frontRight,
    rearRight,
    rearLeft
  };

  enum class ControlMode {
    fieldCentric,
    robotCentric,
  };

  DriveSubsystem();

  void SwerveDrive(const double fwVelocity,
                   const double latVelocity,
                   const double rotateVelocity);

  void Home(const units::degree_t currentAngle);
  void SetFieldOrientation(const units::degree_t);

  void SetControlMode(const ControlMode);

 private:
  void InitializeTurnEncoderAngles();
  void NTUpdate(NetworkTable*, wpi::StringRef, nt::NetworkTableEntry, std::shared_ptr<nt::Value>, int);
  double ModuleDriveSpeed(const units::velocity::feet_per_second_t, const units::velocity::feet_per_second_t, const ctre::phoenix::motorcontrol::Faults);
  wpi::array<frc::SwerveModuleState, 4> RawModuleStates(const double, const double, const double);

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_motorDriveFrontLeft;
  WPI_TalonFX m_motorDriveFrontRight;
  WPI_TalonFX m_motorDriveRearRight;
  WPI_TalonFX m_motorDriveRearLeft;
  WPI_TalonFX m_motorTurnFrontLeft;
  WPI_TalonFX m_motorTurnFrontRight;
  WPI_TalonFX m_motorTurnRearRight;
  WPI_TalonFX m_motorTurnRearLeft;

  CANCoder m_encoderTurnFrontLeft;
  CANCoder m_encoderTurnFrontRight;
  CANCoder m_encoderTurnRearRight;
  CANCoder m_encoderTurnRearLeft;

  frc::ADIS16448_IMU m_IMU;
  frc::Rotation2d m_fieldOrientationOffset;

  units::angular_velocity::degrees_per_second_t m_maxAngularRate;

  std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveKinematicsModel;

  ControlMode m_activeControlMode;
};
