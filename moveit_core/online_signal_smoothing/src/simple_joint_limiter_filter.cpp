#include <moveit/online_signal_smoothing/simple_joint_limiter_filter.h>

namespace online_signal_smoothing
{
namespace
{
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3
constexpr double VELOCITY_EPS_RAD_S = 1e-3;      // rad/s
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops
}

bool SimpleJointLimiterFilterPlugin::initialize(rclcpp::Node::SharedPtr node, const moveit::core::JointModelGroup& group,
                                         size_t num_joints, double timestep_s)
{

  node_ = node;
  // group_ = group;
  num_joints_ = num_joints;
  timestep_s_ = timestep_s;

  const std::vector<std::string>& vars = group.getVariableNames();

  limits_.resize(num_joints_);
  desired_velocity_.resize(num_joints_);
  previous_velocity_vector_.resize(num_joints_);
  joint_names_.resize(num_joints_);

  const moveit::core::RobotModel& rmodel = group.getParentModel();
  for (size_t i = 0; i < num_joints_; ++i)
  {
    limits_[i] = rmodel.getVariableBounds(vars.at(i));
    joint_names_[i] = vars.at(i);
  }
  return true;
};

bool SimpleJointLimiterFilterPlugin::doSmoothing(std::vector<double>& desired_position_vector, 
                                                 const std::vector<double>& current_position_vector,
                                                 std::vector<double>& desired_velocity_vector, 
                                                 const std::vector<double>& current_velocity_vector)
{
  if(first_iteration_) {
    previous_position_vector_ = current_position_vector;
    std::fill(previous_velocity_vector_.begin(), previous_velocity_vector_.end(), 0.0);
    first_iteration_ = false;
  }

  std::vector<double> desired_accel(num_joints_);
  std::vector<double> desired_vel(num_joints_);
  std::vector<double> desired_pos(num_joints_);
  std::vector<bool> pos_limit_trig_jnts(num_joints_, false);
  std::vector<std::string> limited_jnts_vel, limited_jnts_acc;

  bool position_limit_triggered = false;

  for (auto index = 0u; index < num_joints_; ++index)
  {
    desired_pos[index] = desired_position_vector[index];

    // limit position
    if (limits_[index].position_bounded_)
    {
      auto pos = std::max(std::min(limits_[index].max_position_, desired_pos[index]), limits_[index].min_position_);
      if (pos != desired_pos[index])
      {
        pos_limit_trig_jnts[index] = true;
        desired_pos[index] = pos;
      }
    }

    auto delta_pos = desired_pos[index] - current_position_vector[index];
    desired_vel[index] = delta_pos / timestep_s_;

    // limit velocity
    if (limits_[index].velocity_bounded_)
    {
      if (std::abs(desired_vel[index]) > limits_[index].max_velocity_)
      {
        desired_vel[index] = std::copysign(limits_[index].max_velocity_, desired_vel[index]);
        limited_jnts_vel.emplace_back(joint_names_[index]);
      }
    }

    desired_accel[index] = (desired_vel[index] - current_velocity_vector[index]) / timestep_s_;

    // limit acceleration
    if (limits_[index].acceleration_bounded_)
    {
      if (std::abs(desired_accel[index]) > limits_[index].max_acceleration_)
      {
        desired_accel[index] = std::copysign(limits_[index].max_acceleration_, desired_accel[index]);
        desired_vel[index] = current_velocity_vector[index] + desired_accel[index] * timestep_s_;
        // recalc desired position after acceleration limiting
        desired_pos[index] = current_position_vector[index] + 
                             current_velocity_vector[index] * timestep_s_ + 
                             0.5 * desired_accel[index] * timestep_s_ * timestep_s_;
        limited_jnts_acc.emplace_back(joint_names_[index]);
      }
    }

    // Check that stopping distance is within joint limits
    // Slow down all joints at maximum decel if any don't have stopping distance within joint limits
    if (limits_[index].position_bounded_)
    {
      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating.
      // This is a valid assumption if we are not currently at a velocity limit since we are just
      // coming to a rest.
      double stopping_accel = limits_[index].acceleration_bounded_ ? limits_[index].max_acceleration_ :
                                                                     std::abs(desired_vel[index] / timestep_s_);
      double stopping_distance =
          std::abs((-desired_vel[index] * desired_vel[index]) / (2 * stopping_accel));
      // Check that joint limits are beyond stopping_distance and desired_velocity is towards
      // that limit
      if (
        (desired_vel[index] < 0 &&
         (current_position_vector[index] - limits_[index].min_position_ < stopping_distance)) ||
        (desired_vel[index] > 0 &&
         (limits_[index].max_position_ - current_position_vector[index] < stopping_distance)))
      {
        pos_limit_trig_jnts[index] = true;
        position_limit_triggered = true;
      }
    }
  }

  if (position_limit_triggered)
  {
    std::ostringstream ostr;
    for (auto index = 0u; index < num_joints_; ++index)
    {
      // Compute accel to stop
      // Here we aren't explicitly maximally decelerating, but for joints near their limits this
      // should still result in max decel being used
      desired_accel[index] = -current_velocity_vector[index] / timestep_s_;
      if (limits_[index].acceleration_bounded_)
      {
        desired_accel[index] = std::copysign(
          std::min(std::abs(desired_accel[index]), limits_[index].max_acceleration_), desired_accel[index]);
      }

      // Recompute velocity and position
      desired_vel[index] = current_velocity_vector[index] + desired_accel[index] * timestep_s_;
      desired_pos[index] =
        current_position_vector[index] +
        current_velocity_vector[index] * timestep_s_ + 0.5 * desired_accel[index] * timestep_s_ * timestep_s_;
    }
  }

  if (std::count_if(pos_limit_trig_jnts.begin(), pos_limit_trig_jnts.end(), [](bool trig) { return trig; }) > 0)
  {
    std::ostringstream ostr;
    for (auto index = 0u; index < num_joints_; ++index)
    {
      if (pos_limit_trig_jnts[index]) ostr << joint_names_[index] << " ";
    }
    ostr << "\b \b"; // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
                                "Joint(s) [" << ostr.str().c_str() << "] would exceed position limits, limiting");
  }

  if (limited_jnts_vel.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt: limited_jnts_vel) ostr << jnt << " ";
    ostr << "\b \b"; // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
                                "Joint(s) [" << ostr.str().c_str() << "] would exceed velocity limits, limiting");
  }

  if (limited_jnts_acc.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt: limited_jnts_acc) ostr << jnt << " ";
    ostr << "\b \b"; // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
                                "Joint(s) [" << ostr.str().c_str() << "] would exceed acceleration limits, limiting");
  }

  desired_position_vector = previous_position_vector_ = desired_pos;
  desired_velocity_vector = previous_velocity_vector_ = desired_vel;


  return true;
};

bool SimpleJointLimiterFilterPlugin::reset(const std::vector<double>& joint_positions)
{
  first_iteration_ = true;

  return true;
};

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::SimpleJointLimiterFilterPlugin, online_signal_smoothing::SmoothingBaseClass)
