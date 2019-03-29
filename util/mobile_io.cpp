#include "mobile_io.hpp"

#include "ros/ros.h" // For ROS_WARN

#include "hebi_cpp_api/lookup.hpp"

namespace hebi {

MobileIODiff::MobileIODiff(const MobileIOState& prev, const MobileIOState& curr) {
  for (size_t i = 0; i < NumButtons; ++i) {
    if (prev.buttons_[i] && curr.buttons_[i])
      buttons_[i] = ButtonState::On;
    else if (!prev.buttons_[i] && curr.buttons_[i])
      buttons_[i] = ButtonState::ToOn;
    if (!prev.buttons_[i] && !curr.buttons_[i])
      buttons_[i] = ButtonState::Off;
    else if (prev.buttons_[i] && !curr.buttons_[i])
      buttons_[i] = ButtonState::ToOff;
  }
}

MobileIODiff::ButtonState MobileIODiff::get(int button) {
  if (button < 1 || button > NumButtons) {
    assert(false);
    return ButtonState::Off;
  }
  return buttons_[button - 1];
}

std::unique_ptr<MobileIO> MobileIO::create(const std::string& family, const std::string& name) {
  hebi::Lookup lookup;
  std::shared_ptr<hebi::Group> group;
  while (!group) {
    ROS_WARN("Looking for Mobile IO Controller");
    group = lookup.getGroupFromNames({ family }, { name });
  }
  return std::unique_ptr<MobileIO>(new MobileIO(group));
}

MobileIOState MobileIO::getState() {
  // Update if we get another packet from the Mobile IO device
  // (on "failure to get data", just return last data)
  if (group_->getNextFeedback(fbk_)) {
    // We assume the Mobile IO controller is only ever talking to one
    // device at a time...
    auto& f0 = fbk_[0];
    // Update all the buttons in the current state:
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().b().hasInt(i)) {
        current_state_.buttons_[i - 1] = f0.io().b().getInt(i) == 1;
      }
    }
    // And the axes
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().a().hasFloat(i)) {
        current_state_.axes_[i - 1] = f0.io().a().getFloat(i);
      }
    }
  }
  return current_state_;
}
  
MobileIO::MobileIO(std::shared_ptr<hebi::Group> group)
  : group_(group), fbk_(group_->size())
{ }

} // namespace hebi
