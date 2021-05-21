// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

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
    constexpr int driver = 0;
  }
  namespace motor
  {
    constexpr int frontLeftDrive  = 0;
    constexpr int frontRightDrive = 1;
    constexpr int rearRightDrive  = 2;
    constexpr int rearLeftDrive   = 3;

    constexpr int frontLeftTurn   = 4;
    constexpr int frontRightTurn  = 5;
    constexpr int rearRightTurn   = 6;
    constexpr int rearLeftTurn    = 7;
  }
  namespace encoder
  {
    constexpr int frontLeftTurn   = 8;
    constexpr int frontRightTurn  = 9;
    constexpr int rearRightTurn   = 10;
    constexpr int rearLeftTurn    = 11;
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
    constexpr auto frontLeftLatInset = 5.0_in;
    constexpr auto frontLeftLonInset = 5.0_in;
    constexpr auto frontRightLatInset = 5.0_in;
    constexpr auto frontRightLonInset = 5.0_in;
    constexpr auto rearRightLatInset = 5.0_in;
    constexpr auto rearRightLonInset = 5.0_in;
    constexpr auto rearLeftLatInset = 5.0_in;
    constexpr auto rearLeftLonInset = 5.0_in;
  }
}

namespace speedLimits
{
  namespace drive
  {
    constexpr auto maxVelocity = 12_fps;
  }
}
