#pragma once

#include <string>

// TODO(JafarAbdi): This's taken from ros_comm/../names.cpp remove when it's ported
namespace rclcpp
{
namespace names
{
std::string clean(const std::string& name);

std::string append(const std::string& left, const std::string& right);
}  // namespace names
}  // namespace rclcpp
