#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include "ctre/Phoenix.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  enum class ModuleIndex {
    frontLeft,
    frontRight,
    rearRight,
    rearLeft
  };

  DriveSubsystem();

  void SwerveDrive(const double fwVelocity,
                   const double latVelocity,
                   const double rotateVelocity);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
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

  units::angular_velocity::degrees_per_second_t m_maxAngularRate;

  std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveKinematicsModel;
};
