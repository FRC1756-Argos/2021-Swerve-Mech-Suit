#pragma once

#include <array>
#include <frc/GenericHID.h>
#include "argosLib/general/debounceSettings.h"
#include "Vibration.h"

namespace ArgosLib{

class XboxController : public frc::GenericHID {
public:
  enum class Button {
    kA = 1,
    kB = 2,
    kX = 3,
    kY = 4,
    kBumperLeft = 5,
    kBumperRight = 6,
    kBack = 7,
    kStart = 8,
    kStickLeft = 9,
    kStickRight = 10,
    kLeftTrigger = 11,  // virtual button
    kRightTrigger = 12, // virtual button
    kUp = 13,           // virtual button
    kRight = 14,        // virtual button
    kDown = 15,         // virtual button
    kLeft = 16,         // virtual button
    COUNT
  };

  enum class Axis {
    kLeftX = 0,
    kLeftY = 1,
    kLeftTrigger = 2,
    kRightTrigger = 3,
    kRightX = 4,
    kRightY = 5,
    COUNT
  };

  XboxController() = delete;
  XboxController(int port);

  void SetButtonDebounce(Button, debounceSetttings);

  void SwapSettings(XboxController&);

  virtual double GetX(JoystickHand) const override;
  virtual double GetY(JoystickHand) const override;
  double GetTriggerAxis(JoystickHand) const;

  bool GetDebouncedButton(Button);
  bool GetDebouncedButtonPressed(Button);
  bool GetDebouncedButtonReleased(Button);

  bool GetDebouncedButton(std::initializer_list<Button>);
  bool GetDebouncedButtonPressed(std::initializer_list<Button>);
  bool GetDebouncedButtonReleased(std::initializer_list<Button>);

  bool GetRawButton(Button);
  bool GetRawButtonPressed(Button);
  bool GetRawButtonReleased(Button);

  bool GetRawButton(std::initializer_list<Button>);
  bool GetRawButtonPressed(std::initializer_list<Button>);
  bool GetRawButtonReleased(std::initializer_list<Button>);

  void SetVibration(VibrationModel);
  void UpdateVibration();

private:
  struct UpdateStatus {
    bool pressed = false;
    bool released = false;
    bool debouncePress = false;
    bool debounceRelease = false;
    bool rawActive = false;
    bool debounceActive = false;
  };

  struct DPadButtons {
    bool up = false;
    bool right = false;
    bool down = false;
    bool left  = false;
  };

  UpdateStatus UpdateButton(Button);

  DPadButtons GetPOVButtons();

  constexpr static double analogTriggerThresh = 0.5;

  std::array<debounceSetttings, static_cast<int>(Button::COUNT)> m_buttonDebounceSettings;
  std::array<bool, static_cast<int>(Button::COUNT)> m_buttonDebounceStatus;
  std::array<bool, static_cast<int>(Button::COUNT)> m_rawButtonStatus;
  std::array<std::chrono::time_point<std::chrono::steady_clock>, static_cast<int>(Button::COUNT)> m_buttonDebounceStableTime;

  VibrationModel m_vibrationModel;
};

} // namespace ArgosLib
