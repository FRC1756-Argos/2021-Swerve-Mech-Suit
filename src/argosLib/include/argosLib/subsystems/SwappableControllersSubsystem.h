#pragma once

#include <frc2/command/SubsystemBase.h>
#include "argosLib/controller/XboxController.h"

class SwappableControllersSubsystem : public frc2::SubsystemBase {
  public:
    SwappableControllersSubsystem() = delete;
    SwappableControllersSubsystem(int, int);

    void Swap();

    ArgosLib::XboxController& driverController();
    ArgosLib::XboxController& operatorController();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void VibrateAll(ArgosLib::VibrationModel);

  private:
    ArgosLib::XboxController m_driverController;
    ArgosLib::XboxController m_operatorController;
    bool m_swapped;

    void UpdateVibration();
};
