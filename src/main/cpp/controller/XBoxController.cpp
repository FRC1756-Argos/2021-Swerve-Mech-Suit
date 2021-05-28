#include "controller/XboxController.h"

namespace ArgosLib {

XboxController::XboxController(int port) : frc::GenericHID(port) {
  m_buttonDebounceSettings.fill({0_ms, 0_ms});
  m_buttonDebounceStatus.fill(false);
  m_rawButtonStatus.fill(false);
  m_buttonDebounceStableTime.fill(std::chrono::steady_clock::now());
}

void XboxController::SetButtonDebounce(Button targetButton, debounceSetttings newSettings) {
  m_buttonDebounceSettings.at(static_cast<int>(targetButton)) = newSettings;
}

double XboxController::GetX(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == kLeftHand ? Axis::kLeftX : Axis::kRightX));
}

double XboxController::GetY(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == kLeftHand ? Axis::kLeftY : Axis::kRightY));
}

double XboxController::GetTriggerAxis(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == kLeftHand ? Axis::kLeftTrigger : Axis::kRightTrigger));
}

bool XboxController::GetDebouncedButton(Button buttonIdx){
  return UpdateButton(buttonIdx).debounceActive;
}

bool XboxController::GetDebouncedButtonPressed(Button buttonIdx){
  return UpdateButton(buttonIdx).debouncePress;
}

bool XboxController::GetDebouncedButtonReleased(Button buttonIdx){
  return UpdateButton(buttonIdx).debounceRelease;
}

bool XboxController::GetRawButton(Button buttonIdx){
  return UpdateButton(buttonIdx).rawActive;
}

bool XboxController::GetRawButtonPressed(Button buttonIdx){
  return UpdateButton(buttonIdx).pressed;
}

bool XboxController::GetRawButtonReleased(Button buttonIdx){
  return UpdateButton(buttonIdx).released;
}

XboxController::UpdateStatus XboxController::UpdateButton(Button buttonIdx) {
  UpdateStatus retVal;

  bool newVal;
  switch(buttonIdx) {
    case Button::kA:
    case Button::kB:
    case Button::kX:
    case Button::kY:
    case Button::kBumperLeft:
    case Button::kBumperRight:
    case Button::kBack:
    case Button::kStart:
    case Button::kStickLeft:
    case Button::kStickRight:
      newVal = frc::GenericHID::GetRawButton(static_cast<int>(buttonIdx));
      break;
    case Button::kLeftTrigger:
      newVal = GetTriggerAxis(kLeftHand) > analogTriggerThresh;
      break;
    case Button::kRightTrigger:
      newVal = GetTriggerAxis(kRightHand) > analogTriggerThresh;
      break;
    case Button::kUp:
      newVal = GetPOVButtons().up;
      break;
    case Button::kRight:
      newVal = GetPOVButtons().right;
      break;
    case Button::kDown:
      newVal = GetPOVButtons().down;
      break;
    case Button::kLeft:
      newVal = GetPOVButtons().left;
      break;
    case Button::COUNT:
    default:
      throw std::runtime_error("Invalid button index selected");
  }

  const bool prevRawVal = m_rawButtonStatus.at(static_cast<int>(buttonIdx));
  const bool activeDebounceVal = m_buttonDebounceStatus.at(static_cast<int>(buttonIdx));
  const auto activeStableTime = m_buttonDebounceStableTime.at(static_cast<int>(buttonIdx));
  const auto curTime = std::chrono::steady_clock::now();

  if(newVal != activeDebounceVal)
  {
    const auto timeSinceStable = units::millisecond_t{static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(curTime - activeStableTime).count())};
    if(newVal) {
      if(timeSinceStable > m_buttonDebounceSettings.at(static_cast<int>(buttonIdx)).activateTime) {
        retVal.debouncePress = true;
        m_buttonDebounceStableTime.at(static_cast<int>(buttonIdx)) = curTime;
        m_buttonDebounceStatus.at(static_cast<int>(buttonIdx)) = newVal;
      }
    } else {
      if(timeSinceStable > m_buttonDebounceSettings.at(static_cast<int>(buttonIdx)).clearTime) {
        retVal.debounceRelease = true;
        m_buttonDebounceStableTime.at(static_cast<int>(buttonIdx)) = curTime;
        m_buttonDebounceStatus.at(static_cast<int>(buttonIdx)) = newVal;
      }
    }
  }

  retVal.pressed = newVal && !prevRawVal;
  retVal.released = !newVal && prevRawVal;
  retVal.rawActive = newVal;
  retVal.debounceActive = m_buttonDebounceStatus.at(static_cast<int>(buttonIdx));

  return retVal;
}

XboxController::DPadButtons XboxController::GetPOVButtons() {
  const auto povAngle{GetPOV()};
  return DPadButtons{(povAngle >= 0   && povAngle <= 45) || povAngle >= 315,
                      povAngle >= 45  && povAngle <= 135,
                      povAngle >= 135 && povAngle <= 225,
                      povAngle >= 225 && povAngle <= 315};
}

} // namespace ArgosLib
