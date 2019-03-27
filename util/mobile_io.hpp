#pragma once

#include <memory>
#include <hebi_cpp_api/group.hpp>
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {

static constexpr size_t NumButtons = 8;

// The current state at any time
struct MobileIOState {
  std::array<bool, NumButtons> buttons_;
  std::array<float, NumButtons> axes_;
};

// Difference between two IO states, useful for checking to see if a button 
// has been pressed.
struct MobileIODiff {
  MobileIODiff(const MobileIOState& prev, const MobileIOState& current);

  enum class ButtonState {
    Off, On, // These occur if last + current state are the same
    ToOff, ToOn // Edge triggers; these occur if last + current state are different
  };

  // Note: one-indexed to match buttons on the screen
  ButtonState get(int button);

private:
  std::array<ButtonState, NumButtons> buttons_;
};

// Wrapper around a mobile IO controller
class MobileIO {
public:

  static std::unique_ptr<MobileIO> create(const std::string& family, const std::string& name);
	
  MobileIOState getState();

private:
  MobileIO(std::shared_ptr<hebi::Group>);

  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback fbk_;
  MobileIOState current_state_;
};

}


