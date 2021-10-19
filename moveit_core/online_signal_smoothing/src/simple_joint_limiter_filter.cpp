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
  
  const moveit::core::RobotModel& rmodel = group.getParentModel();
  for (size_t i = 0; i < num_joints_; ++i)
  {
    limits_[i] = rmodel.getVariableBounds(vars.at(i));
  }
  return true;
};

bool SimpleJointLimiterFilterPlugin::doSmoothing(std::vector<double>& desired_position_vector, std::vector<double>& current_position_vector)
{
  if(first_iteration_) {
    previous_position_vector_ = current_position_vector;
    std::fill(previous_velocity_vector_.begin(), previous_velocity_vector_.end(), 0.0);
    first_iteration_ = false;
  }

  // Compute necessary velocity
  for (auto index = 0u; index < num_joints_; ++index)
  {
    desired_velocity_.at(index) = (desired_position_vector.at(index) - current_position_vector.at(index)) / timestep_s_;
  }
  // Compute current velocity
  for (auto index = 0u; index < num_joints_; ++index)
  {
    previous_velocity_vector_.at(index) = (current_position_vector.at(index) - previous_position_vector_.at(index)) / timestep_s_;
  }

  // Clamp velocities to limits
  for (auto index = 0u; index < num_joints_; ++index)
  {
    if (limits_[index].velocity_bounded_)
    {
      if (std::abs(desired_velocity_[index]) > limits_[index].max_velocity_)
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed velocity limits, limiting");
        desired_velocity_[index] =
          copysign(limits_[index].max_velocity_, desired_velocity_[index]);
        double accel =
          (desired_velocity_[index] - previous_velocity_vector_[index]) /
          timestep_s_;
        // Recompute position
        desired_position_vector[index] =
          current_position_vector[index] +
          previous_velocity_vector_[index] * timestep_s_ +
          0.5 * accel * timestep_s_ * timestep_s_;
      }
    }
  }

  // Clamp acclerations to limits
  for (auto index = 0u; index < num_joints_; ++index)
  {
    if (limits_[index].acceleration_bounded_)
    {
      double accel =
        (desired_velocity_[index] - previous_velocity_vector_[index]) /
        timestep_s_;
      if (std::abs(accel) > limits_[index].max_acceleration_)
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed acceleration limits, limiting");
        desired_velocity_[index] =
          previous_velocity_vector_[index] +
          copysign(limits_[index].max_acceleration_, accel) * timestep_s_;
        // Recompute position
        desired_position_vector[index] =
          current_position_vector[index] +
          previous_velocity_vector_[index] * timestep_s_ +
          0.5 * copysign(limits_[index].max_acceleration_, accel) * timestep_s_ * timestep_s_;
      }
    }
  }

  // Check that stopping distance is within joint limits
  // - In joint mode, slow down only joints whose stopping distance isn't inside joint limits,
  // at maximum decel
  // - In Cartesian mode, slow down all joints at maximum decel if any don't have stopping distance
  // within joint limits
  bool position_limit_triggered = false;
  for (auto index = 0u; index < num_joints_; ++index)
  {
    if (limits_[index].acceleration_bounded_)
    {
      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating.
      // This is a valid assumption if we are not currently at a velocity limit since we are just
      // coming to a rest.
      double stopping_distance = std::abs(
        (-desired_velocity_[index] * desired_velocity_[index]) /
        (2 * limits_[index].max_acceleration_));
      // Check that joint limits are beyond stopping_distance and desired_velocity is towards
      // that limit
      // TODO(anyone): Should we consider sign on acceleration here?
      if (
        (desired_velocity_[index] < 0 &&
         (current_position_vector[index] - limits_[index].min_position_ <
          stopping_distance)) ||
        (desired_velocity_[index] > 0 &&
         (limits_[index].max_position_ - current_position_vector[index] <
          stopping_distance)))
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed position limits, limiting");
        position_limit_triggered = true;

        // We will limit all joints
        break;
      }
    }
  }

  if (position_limit_triggered)
  {
    // In Cartesian admittance mode, stop all joints if one would exceed limit
    for (auto index = 0u; index < num_joints_; ++index)
    {
      if (limits_[index].acceleration_bounded_)
      {
        // Compute accel to stop
        // Here we aren't explicitly maximally decelerating, but for joints near their limits this
        // should still result in max decel being used
        double accel_to_stop = -previous_velocity_vector_[index] / timestep_s_;
        double limited_accel = copysign(
          std::min(std::abs(accel_to_stop), limits_[index].max_acceleration_), accel_to_stop);

        desired_velocity_[index] =
          previous_velocity_vector_[index] + limited_accel * timestep_s_;
        // Recompute position
        desired_position_vector[index] =
          current_position_vector[index] +
          previous_velocity_vector_[index] * timestep_s_ +
          0.5 * limited_accel * timestep_s_ * timestep_s_;
      }
    }
  }

  previous_position_vector_ = current_position_vector;
    
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
