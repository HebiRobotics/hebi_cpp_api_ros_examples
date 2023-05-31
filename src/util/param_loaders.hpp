#pragma once

#include <type_traits>
#include "rclcpp/rclcpp.hpp"


/*template <typename T>
rclcpp::ParameterType getScalarParamType(T t) {
  if constexpr(std::is_same<T, bool>::value) {

  } else if constexpr(std::is_same<T, int>::value) {

  } else if constexpr(std::is_same<T, double>::value) {

  } else if constexpr(std::is_same<T, std::string>::value) {

  }
}

std::string loadScalarParamWarn(std::shared_ptr<rclcpp::Node> node, const std::string name, std::string default_value) {

  node->declare_parameter<std::string>(name);

  auto val = node->get_parameter(name);
  if (val.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find/read '" << name << "' string parameter; defaulting to '" << default_value << "'");
    return default_value;
  }
  return val.as_string();
}

rclcpp::ParameterType getScalarParamType(bool t) {
 return rclcpp::ParameterType::PARAMETER_BOOL; 
}

rclcpp::ParameterType getScalarParamType(int t) {
 return rclcpp::ParameterType::PARAMETER_INTEGER; 
}

rclcpp::ParameterType getScalarParamType(double t) {
 return rclcpp::ParameterType::PARAMETER_DOUBLE; 
}

rclcpp::ParameterType getScalarParamType(std::string t) {
 return rclcpp::ParameterType::PARAMETER_STRING; 
}
*/


int loadScalarParamWarn(std::shared_ptr<rclcpp::Node> node, const std::string name, int default_value) {

  node->declare_parameter<int>(name);

  auto val = node->get_parameter(name);
  if (val.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find/read '" << name << "' parameter; defaulting to '" << default_value << "'");
    return default_value;
  }
  return val.as_int();
}

double loadScalarParamWarn(std::shared_ptr<rclcpp::Node> node, const std::string name, double default_value) {

  node->declare_parameter<double>(name);

  auto val = node->get_parameter(name);
  if (val.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find/read '" << name << "' parameter; defaulting to '" << default_value << "'");
    return default_value;
  }
  return val.as_double();
}


std::string loadScalarParamWarn(std::shared_ptr<rclcpp::Node> node, const std::string name, const char* default_value) {

  node->declare_parameter<std::string>(name);

  auto val = node->get_parameter(name);
  if (val.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find/read '" << name << "' parameter; defaulting to '" << default_value << "'");
    return default_value;
  }
  return val.as_string();
}


bool loadScalarParamWarn(std::shared_ptr<rclcpp::Node> node, const std::string name, bool default_value) {

  node->declare_parameter<bool>(name);

  auto val = node->get_parameter(name);
  if (val.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find/read '" << name << "' parameter; defaulting to '" << default_value << "'");
    return default_value;
  }
  return val.as_bool();
}