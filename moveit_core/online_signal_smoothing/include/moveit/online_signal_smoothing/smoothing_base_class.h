/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andy Zelenak
   Description: Defines a pluginlib interface for smoothing algorithms.
 */

#pragma once

#include <moveit/robot_model/robot_model.h>

namespace online_signal_smoothing
{
class SmoothingBaseClass
{
public:
  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, typically used for parameter retrieval
   * @param group typically used to retrieve pos/vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
   * @param timestep_s control loop period [seconds]
   * @return True if initialization was successful
   */
  // TODO: Remove num_joints as it can be derived from group size
  virtual bool initialize(rclcpp::Node::SharedPtr node, const moveit::core::JointModelGroup& group,
                          size_t num_joints, double timestep_s) = 0;

  /**
   * Smooth an array of joint position deltas (update desired position and velocity)
   * @param desired_position_vector array of joint position commands
   * @param current_position_vector array of current joint positions
   * @param desired_velocity_vector array of joint velocity commands
   * @param current_velocity_vector array of current joint velocity
   * @return True if initialization was successful
   */
  virtual bool doSmoothing(std::vector<double>& desired_position_vector, 
                           const std::vector<double>& current_position_vector, 
                           std::vector<double>& desired_velocity_vector, 
                           const std::vector<double>& current_velocity_vector) = 0;

  /**
   * Reset to a given joint state
   * @param joint_positions reset the filters to these joint positions
   * @return True if reset was successful
   */
  virtual bool reset(const std::vector<double>& joint_positions) = 0;
};
}  // namespace online_signal_smoothing
