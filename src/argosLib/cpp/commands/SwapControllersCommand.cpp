#include "argosLib/commands/SwapControllersCommand.h"

SwapControllersCommand::SwapControllersCommand(SwappableControllersSubsystem* controllers) : m_pControllerSubsystem(controllers) {}

void SwapControllersCommand::Initialize() {
  m_pControllerSubsystem->VibrateAll(ArgosLib::VibrationConstant(1));
}

void SwapControllersCommand::End(bool) {
  m_pControllerSubsystem->Swap();
  m_pControllerSubsystem->VibrateAll(ArgosLib::VibrationOff());
}
