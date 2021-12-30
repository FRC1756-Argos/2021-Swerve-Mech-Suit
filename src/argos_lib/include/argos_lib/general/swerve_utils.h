/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace argos_lib {
  namespace swerve {

    /**
 * @brief Finds closest angle alias of desiredAngle relative to referencedAngle.
 *        All inputs are normalized so ranges are unbounded.
 *
 * @param desiredAngle Angle to find alias for
 * @param referenceAngle Defines desired alias range
 * @return Angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
 */
    units::degree_t NearestAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
 * @brief Finds closest angle alias of a vector 180-degrees offset from desiredAngle relative to referencedAngle.
 *        All inputs are normalized so ranges are unbounded.
 *
 * @param desiredAngle Angle to find 180-degree offset alias for
 * @param referenceAngle Defines desired alias range
 * @return Offset angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
 */
    units::degree_t InvertedAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
 * @brief Normalize angle to specified range
 *
 * @param inVal Angle to constrain
 * @param minVal Normalization lower bound
 * @param maxVal Normalization upper bound
 * @return Normalized value in range [minVal, maxVal]
 */
    units::degree_t ConstrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal);
    /**
 * @brief Normalize angle to specified range
 *
 * @param inVal Angle to constrain (degrees)
 * @param minVal Normalization lower bound (degrees)
 * @param maxVal Normalization upper bound (degrees)
 * @return Normalized value in range [minVal, maxVal] (degrees)
 */
    double ConstrainAngle(double inVal, double minVal, double maxVal);

    /**
 * @brief Optimize swerve module to minimize rotations and drive direction changes
 *
 * @param desiredState Requested state.  Output must result in same motion
 * @param currentModuleAngle Module rotation angle in relative or absolute position
 * @param currentModuleAngularRate Current module rotation speed.  To prevent rapid changes in rotation direction.
 * @param currentModuleDriveVel Current module drive velocity.  To prevent rapid changes in drive motor velocity.
 * @return Optimized swerve module state that results in same motion as desiredState
 */
    frc::SwerveModuleState Optimize(frc::SwerveModuleState desiredState,
                                    units::degree_t currentModuleAngle,
                                    units::degrees_per_second_t currentModuleAngularRate,
                                    units::feet_per_second_t currentModuleDriveVel);

  }  // namespace swerve
}  // namespace argos_lib
