/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <adi/ADIS16448_IMU.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <frc/SPI.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "argos_lib/general/interpolation.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace address {
  namespace joystick {
    constexpr int driver = 0;
    constexpr int secondary = 1;
  }  // namespace joystick
  namespace motor {
    constexpr int intake = 3;

    constexpr int frontLeftDrive = 10;
    constexpr int frontRightDrive = 11;
    constexpr int rearRightDrive = 12;
    constexpr int rearLeftDrive = 13;

    constexpr int frontLeftTurn = 14;
    constexpr int frontRightTurn = 15;
    constexpr int rearRightTurn = 16;
    constexpr int rearLeftTurn = 17;
  }  // namespace motor
  namespace encoder {
    constexpr int frontLeftTurn = 1;
    constexpr int frontRightTurn = 2;
    constexpr int rearRightTurn = 3;
    constexpr int rearLeftTurn = 4;
  }  // namespace encoder
}  // namespace address

namespace measureUp {
  namespace chassis {
    constexpr units::inch_t width{28.0};
    constexpr units::inch_t length{28.0};
  }  // namespace chassis
  namespace drive {
    constexpr auto frontLeftLatInset = 4.0_in;
    constexpr auto frontLeftLonInset = 4.0_in;
    constexpr auto frontRightLatInset = 4.0_in;
    constexpr auto frontRightLonInset = 4.0_in;
    constexpr auto rearRightLatInset = 4.0_in;
    constexpr auto rearRightLonInset = 4.0_in;
    constexpr auto rearLeftLatInset = 4.0_in;
    constexpr auto rearLeftLonInset = 4.0_in;

    constexpr auto wheelDiameter = 4.0_in;
    constexpr auto wheelCircumference = wheelDiameter * M_PI;
  }  // namespace drive
  namespace sensorConversion {
    namespace swerveRotate {
      constexpr auto ticksPerDegree = 4096.0 / 360.0;
      constexpr auto toAngle(double sensorVal) { return units::make_unit<units::degree_t>(sensorVal / ticksPerDegree); }
      constexpr auto fromAngle(units::degree_t angVal) { return angVal.to<double>() * ticksPerDegree; }
      constexpr auto toAngVel(double sensorVal) {
        return units::make_unit<units::degrees_per_second_t>(sensorVal / ticksPerDegree);
      }
      constexpr auto fromAngVel(units::degree_t angVelVal) { return angVelVal.to<double>() * ticksPerDegree; }
    }  // namespace swerveRotate
    namespace swerveDrive {
      constexpr auto toDist(double sensorVal) { return measureUp::drive::wheelCircumference / 8.16 * sensorVal; }
      constexpr auto fromDist(units::inch_t distVal) {
        return (distVal * 8.16 / measureUp::drive::wheelCircumference).to<double>();
      }
      constexpr auto toVel(double sensorVal) {
        return measureUp::drive::wheelCircumference / 8.16 / 100_ms * sensorVal;
      }
      constexpr auto fromVel(units::feet_per_second_t velValue) {
        return (velValue * 8.16 * 100_ms / measureUp::drive::wheelCircumference).to<double>();
      }
    }  // namespace swerveDrive
  }    // namespace sensorConversion
}  // namespace measureUp

namespace speedLimits {
  namespace drive {
    constexpr auto maxVelocity = 12_fps;
  }  // namespace drive
}  // namespace speedLimits

namespace controlLoop {
  namespace drive {
    namespace rotate {
      constexpr double kP = 1.4;
      constexpr double kI = 0.01;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 0.0;
    }  // namespace rotate
  }    // namespace drive
}  // namespace controlLoop

namespace sensorConfig {
  namespace drive {
    struct frontLeftTurn {
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct frontRightTurn {
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct rearRightTurn {
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct rearLeftTurn {
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    namespace IMU {
      constexpr auto yawAxis = frc::ADIS16448_IMU::IMUAxis::kZ;
      constexpr auto port = frc::SPI::Port::kMXP;
      constexpr auto calTime = 5.0_s;
    }  // namespace IMU
  }    // namespace drive
}  // namespace sensorConfig

namespace motorConfig {
  namespace drive {
    struct frontLeftDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct frontRightDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct rearRightDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct rearLeftDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct frontLeftTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoder::frontLeftTurn;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct frontRightTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoder::frontRightTurn;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct rearRightTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoder::rearRightTurn;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct rearLeftTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoder::rearLeftTurn;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
  }  // namespace drive
  namespace intake {
    struct intakeMotor {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = 11.0_V;
    };
  }  // namespace intake
}  // namespace motorConfig

namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveLongSpeed{InterpMapPoint{-1.0, 0.6},
                                                       InterpMapPoint{-0.75, 0.4},
                                                       InterpMapPoint{-0.15, 0.0},
                                                       InterpMapPoint{0.15, 0.0},
                                                       InterpMapPoint{0.75, -0.4},
                                                       InterpMapPoint{1.0, -0.6}};
  [[maybe_unused]] constexpr std::array driveLatSpeed{InterpMapPoint{-1.0, -0.6},
                                                      InterpMapPoint{-0.75, -0.4},
                                                      InterpMapPoint{-0.15, 0.0},
                                                      InterpMapPoint{0.15, 0.0},
                                                      InterpMapPoint{0.75, 0.4},
                                                      InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};
}  // namespace controllerMap

namespace ntKeys {
  constexpr auto tableName = "Argos";
  namespace subsystemDrive {
    namespace homePosition {
      constexpr auto turnFrontLeft = "drive/homing/turnFrontLeft";
      constexpr auto turnFrontRight = "drive/homing/turnFrontRight";
      constexpr auto turnRearRight = "drive/homing/turnRearRight";
      constexpr auto turnRearLeft = "drive/homing/turnRearLeft";
    }  // namespace homePosition
    namespace tuning {
      namespace turn {
        constexpr auto subtableName = "drive/tuning/turn";
        constexpr auto kP = "drive/tuning/turn/kP";
        constexpr auto kI = "drive/tuning/turn/kI";
        constexpr auto kD = "drive/tuning/turn/kD";
        constexpr auto kF = "drive/tuning/turn/kF";
        constexpr auto iZone = "drive/tuning/turn/iZone";
        constexpr auto allowableError = "drive/tuning/turn/allowableError";
      }  // namespace turn
    }    // namespace tuning
  }      // namespace subsystemDrive
}  // namespace ntKeys
