/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "argosLib/subsystems/SwappableControllersSubsystem.h"

class SwapControllersCommand : public frc2::CommandHelper<frc2::CommandBase, SwapControllersCommand> {
 public:
  explicit SwapControllersCommand(SwappableControllersSubsystem*);

  void Initialize() override;
  void End(bool) override;

 private:
  SwappableControllersSubsystem* m_pControllerSubsystem;
};
