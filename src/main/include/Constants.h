// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include "general/interpolation.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace address
{
  namespace joystick
  {
    constexpr int driver    = 0;
    constexpr int secondary = 1;
  }
  namespace motor
  {
    constexpr int frontLeftDrive  = 10;
    constexpr int frontRightDrive = 11;
    constexpr int rearRightDrive  = 12;
    constexpr int rearLeftDrive   = 13;

    constexpr int frontLeftTurn   = 14;
    constexpr int frontRightTurn  = 15;
    constexpr int rearRightTurn   = 16;
    constexpr int rearLeftTurn    = 17;
  }
  namespace encoder
  {
    constexpr int frontLeftTurn   = 1;
    constexpr int frontRightTurn  = 2;
    constexpr int rearRightTurn   = 3;
    constexpr int rearLeftTurn    = 4;
  }
}

namespace measureUp
{
  namespace chassis
  {
    constexpr units::inch_t width{28.0};
    constexpr units::inch_t length{28.0};
  }
  namespace drive
  {
    constexpr auto frontLeftLatInset  = 4.0_in;
    constexpr auto frontLeftLonInset  = 4.0_in;
    constexpr auto frontRightLatInset = 4.0_in;
    constexpr auto frontRightLonInset = 4.0_in;
    constexpr auto rearRightLatInset  = 4.0_in;
    constexpr auto rearRightLonInset  = 4.0_in;
    constexpr auto rearLeftLatInset   = 4.0_in;
    constexpr auto rearLeftLonInset   = 4.0_in;

    constexpr auto wheelDiameter      = 4.0_in;
    constexpr auto wheelCircumference = wheelDiameter * M_PI;
  }
  namespace sensorConversion
  {
    namespace swerveRotate
    {
      constexpr auto ticksPerDegree = 4096.0 / 360.0;
      constexpr auto toAngle(double sensorVal) { return units::make_unit<units::degree_t>(sensorVal / ticksPerDegree); }
      constexpr auto fromAngle(units::degree_t angVal) { return angVal.to<double>() * ticksPerDegree; }
      constexpr auto toAngVel(double sensorVal) { return units::make_unit<units::degrees_per_second_t>(sensorVal / ticksPerDegree); }
      constexpr auto fromAngVel(units::degree_t angVelVal) { return angVelVal.to<double>() * ticksPerDegree; }
    }
    namespace swerveDrive
    {
      constexpr auto toDist(double sensorVal) { return measureUp::drive::wheelCircumference / 8.16 * sensorVal; }
      constexpr auto fromDist(units::inch_t distVal) { return (distVal * 8.16 / measureUp::drive::wheelCircumference).to<double>(); }
      constexpr auto toVel(double sensorVal) { return measureUp::drive::wheelCircumference / 8.16 / 100_ms * sensorVal; }
      constexpr auto fromVel(units::feet_per_second_t velValue) { return (velValue * 8.16  * 100_ms / measureUp::drive::wheelCircumference).to<double>(); }
    }
  }
}

namespace speedLimits
{
  namespace drive
  {
    constexpr auto maxVelocity = 12_fps;
  }
}

namespace controlLoop
{
  namespace drive
  {
    namespace rotate
    {
      constexpr double kP = 1.2;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 0.0;
      constexpr double allowableError = 0.0;
    }
  }
}

namespace sensorConfig
{
  namespace drive
  {
    namespace frontLeftTurn
    {
      constexpr bool direction = false;
      constexpr auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr auto magOffset = 0;
    }
    namespace frontRightTurn
    {
      constexpr bool direction = false;
      constexpr auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr auto magOffset = 0;
    }
    namespace rearRightTurn
    {
      constexpr bool direction = false;
      constexpr auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr auto magOffset = 0;
    }
    namespace rearLeftTurn
    {
      constexpr bool direction = false;
      constexpr auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr auto magOffset = 0;
    }
  }
}

namespace motorConfig
{
  namespace drive
  {
    namespace frontLeftDrive
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace frontRightDrive
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace rearRightDrive
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace rearLeftDrive
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace frontLeftTurn
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace frontRightTurn
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace rearRightTurn
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
    namespace rearLeftTurn
    {
      constexpr bool inverted      = false;
      constexpr bool sensorPhase   = false;
      constexpr auto neutralMode   = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr double voltCompSat = 11.0;
    }
  }
}

namespace controllerMap
{
  [[maybe_unused]] constexpr std::array driveLongSpeed{ interpMapPoint{-1.0,   0.5},
                                                        interpMapPoint{-0.15,  0.0},
                                                        interpMapPoint{ 0.15,  0.0},
                                                        interpMapPoint{ 1.0,  -0.5} };
  [[maybe_unused]] constexpr std::array driveLatSpeed{ interpMapPoint{-1.0,  -0.5},
                                                       interpMapPoint{-0.15,  0.0},
                                                       interpMapPoint{ 0.15,  0.0},
                                                       interpMapPoint{ 1.0,   0.5} };
  [[maybe_unused]] constexpr std::array driveRotSpeed{ interpMapPoint{-1.0,  -0.5},
                                                       interpMapPoint{-0.15,  0.0},
                                                       interpMapPoint{ 0.15,  0.0},
                                                       interpMapPoint{ 1.0,   0.5} };
}

namespace ntKeys
{
  constexpr auto tableName = "Argos";
  namespace subsystemDrive
  {
    namespace homePosition
    {
      constexpr auto turnFrontLeft  = "drive/homing/turnFrontLeft";
      constexpr auto turnFrontRight = "drive/homing/turnFrontRight";
      constexpr auto turnRearRight  = "drive/homing/turnRearRight";
      constexpr auto turnRearLeft   = "drive/homing/turnRearLeft";
    }
    namespace tuning
    {
      namespace turn
      {
        constexpr auto subtableName = "drive/tuning/turn";
        constexpr auto kP = "drive/tuning/turn/kP";
        constexpr auto kI = "drive/tuning/turn/kI";
        constexpr auto kD = "drive/tuning/turn/kD";
        constexpr auto kF = "drive/tuning/turn/kF";
        constexpr auto iZone = "drive/tuning/turn/iZone";
        constexpr auto allowableError = "drive/tuning/turn/allowableError";
      }
    }
  }
}
