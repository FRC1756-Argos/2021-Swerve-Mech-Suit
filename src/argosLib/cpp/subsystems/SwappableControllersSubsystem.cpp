#include "argosLib/subsystems/SwappableControllersSubsystem.h"

SwappableControllersSubsystem::SwappableControllersSubsystem(int driverControllerPort, int operatorControllerPort) : m_driverController(driverControllerPort),
                                                                                                                     m_operatorController(operatorControllerPort),
                                                                                                                     m_swapped(false) {}

void SwappableControllersSubsystem::Swap() {
  m_driverController.SwapSettings(m_operatorController);
  m_swapped = !m_swapped;
}

ArgosLib::XboxController& SwappableControllersSubsystem::driverController() {
  return m_swapped ? m_operatorController : m_driverController;
}
ArgosLib::XboxController& SwappableControllersSubsystem::operatorController() {
  return m_swapped ? m_driverController : m_operatorController;
}

/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void SwappableControllersSubsystem::Periodic() {
  UpdateVibration();
}

void SwappableControllersSubsystem::VibrateAll(ArgosLib::VibrationModel newModel) {
  m_driverController.SetVibration(newModel);
  m_operatorController.SetVibration(newModel);
}

void SwappableControllersSubsystem::UpdateVibration() {
  m_driverController.UpdateVibration();
  m_operatorController.UpdateVibration();
}
