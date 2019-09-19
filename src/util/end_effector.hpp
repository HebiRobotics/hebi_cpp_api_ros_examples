#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"

namespace hebi {
namespace arm {

// This is a general end effector that can be added to the end of the arm.
// Override this class to create end effectors for particular purposes.
class EndEffectorBase {
public:
  EndEffectorBase() = default;
  // Updates feedback and sets aux state.
  virtual bool update(Eigen::VectorXd& aux_state) { return true; }
  // Sends command
  virtual bool send() { return true; }
};

// An end effector with "n" commandable states.
// Override this abstract base class to create a fully functioning end
// effector (e.g., the "EffortEndEffector" below).
template <size_t AuxSize>
class EndEffector : public EndEffectorBase {

public:
  // Updates feedback and sets aux state.
  // Note: parallel grippers should have aux_state size of AuxSize or 0.
  // State of "0" indicates no change. Values of "nan" also indicate
  // no change.
  // Invalid inputs result in a "false" return value, with no
  // command set.
  bool update(Eigen::VectorXd& aux_state) override {
    // Check for valid aux state:
    auto n = aux_state.size();
    if (n != AuxSize && n != 0)
      return false;

    // Set aux state when given:
    if (aux_state.size() == AuxSize)
    {
      for (size_t i = 0; i < AuxSize; ++i)
      {
        if (!std::isnan(aux_state[i]))
          setCommand(i, aux_state[i]);
      }
    }

    return group_->getNextFeedback(feedback_);
  }

  // Sends command to gripper.
  bool send() override {
    return group_->sendCommand(command_);
  }

protected:
  // Typed setters depending on class type
  virtual void setCommand(size_t index, double value) = 0;

  static std::shared_ptr<hebi::Group> getGroup(const std::vector<std::string>& families, const std::vector<std::string>& names) {
    Lookup lookup;
    auto group = lookup.getGroupFromNames(families, names);
    if (!group || group->size() != AuxSize)
      return nullptr;
    return group;
  }

  EndEffector(std::shared_ptr<hebi::Group> group) : command_(AuxSize), group_(group), feedback_(AuxSize) { }

  hebi::GroupCommand command_;

private:
  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback feedback_;
};

// A general effort-controlled gripper, sending commands to HEBI modules.  This
// templated class can work with "n" modules, although "1" is the most common case.
// 
// Example usage for effort-controller gripper:
// 
// auto end_effector = EndEffector<1, EndEffectorCommand::Effort>::create("HEBI", "Gripper");
// Eigen::VectorXd aux_state(1);
// aux_state.setConstant(0);
// while(true) {
//   aux_state = updateAuxState(); // Fill in this state from somewhere...
//   end_effector->update(new_aux_state);
//   end_effector->send();
// }
//
// Note -- you will probably want to check the return values of update and send to
// verify module connection is stable.
// 
// Note that this is designed to be used with the arm API, but can also be used independently
// if desired.
template <size_t AuxSize>
class EffortEndEffector : public EndEffector<AuxSize> {

public:
  // Create a gripper group for n modules, using the modules' famil(ies) and name(s).
  static std::unique_ptr<EffortEndEffector> create(const std::string& family, const std::string& name) {
    return create(std::vector<std::string> { family }, std::vector<std::string> { name });
  }

  // Create a gripper group for n modules, using the modules' famil(ies) and name(s).
  static std::unique_ptr<EffortEndEffector> create(const std::vector<std::string>& families, const std::vector<std::string>& names) {
    if (auto group = EndEffector<AuxSize>::getGroup(families, names))
      return std::unique_ptr<EffortEndEffector>(new EffortEndEffector(group));
    return nullptr;
  }

  Eigen::VectorXd getState() {
    return this->feedback_.getEfforts();
  }

protected:
  void setCommand(size_t index, double value) override {
    this->command_[index].actuator().effort().set(value);
  }

private:
  EffortEndEffector(std::shared_ptr<hebi::Group> group) : EndEffector<AuxSize>(group) { }
};

} // namespace arm
} // namespace hebi