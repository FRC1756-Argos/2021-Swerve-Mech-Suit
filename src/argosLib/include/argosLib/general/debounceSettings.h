/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <Units/time.h>

struct debounceSetttings {
  units::millisecond_t activateTime;
  units::millisecond_t clearTime;
};
