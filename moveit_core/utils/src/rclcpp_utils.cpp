#include <moveit/utils/rclcpp_utils.h>

namespace rclcpp
{
namespace names
{
std::string clean(const std::string& name)
{
  std::string clean = name;

  size_t pos = clean.find("//");
  while (pos != std::string::npos)
  {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
  }

  if (!name.empty() && *clean.rbegin() == '/')
  {
    clean.erase(clean.size() - 1, 1);
  }

  return clean;
}

std::string append(const std::string& left, const std::string& right)
{
  return clean(left + "/" + right);
}
}  // namespace names
}  // namespace rclcpp
