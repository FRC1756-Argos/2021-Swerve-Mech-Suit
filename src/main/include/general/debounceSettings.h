#pragma once

#include <Units/time.h>

struct debounceSetttings {
  units::millisecond_t activateTime;
  units::millisecond_t clearTime;
};
