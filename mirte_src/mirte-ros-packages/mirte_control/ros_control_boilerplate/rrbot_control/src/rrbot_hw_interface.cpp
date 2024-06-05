/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include "ros/ros.h"
#include <chrono>
#include <iostream>
#include <mirte_msgs/ServoPosition.h>
#include <mirte_msgs/SetServoAngle.h>
#include <rrbot_control/rrbot_hw_interface.h>

namespace rrbot_control {

double data[4] = {0.0, 0.0, 0.0, 0.0};
bool servo_init[4] = {false, false, false, false};
bool initialized = false;
int init_steps = 0;

ros::Subscriber sub0;
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber sub3;

ros::ServiceClient client0;
ros::ServiceClient client1;
ros::ServiceClient client2;
ros::ServiceClient client3;

mirte_msgs::SetServoAngle srv0;
mirte_msgs::SetServoAngle srv1;
mirte_msgs::SetServoAngle srv2;
mirte_msgs::SetServoAngle srv3;

void callbackJoint0(const mirte_msgs::ServoPosition::ConstPtr &msg) {
  data[0] = msg->angle;
  servo_init[0] = true;
}

void callbackJoint1(const mirte_msgs::ServoPosition::ConstPtr &msg) {
  data[1] = msg->angle;
  servo_init[1] = true;
}

void callbackJoint2(const mirte_msgs::ServoPosition::ConstPtr &msg) {
  data[2] = msg->angle;
  servo_init[2] = true;
}

void callbackJoint3(const mirte_msgs::ServoPosition::ConstPtr &msg) {
  data[3] = msg->angle;
  servo_init[3] = true;
}

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
  this->connectServices();

  sub0 = nh.subscribe("/mirte/servos/servoRot/position", 1,
                      rrbot_control::callbackJoint0);
  sub1 = nh.subscribe("/mirte/servos/servoShoulder/position", 1,
                      rrbot_control::callbackJoint1);
  sub2 = nh.subscribe("/mirte/servos/servoElbow/position", 1,
                      rrbot_control::callbackJoint2);
  sub3 = nh.subscribe("/mirte/servos/servoWrist/position", 1,
                      rrbot_control::callbackJoint3);

  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
}

void RRBotHWInterface::connectServices() {
  ROS_INFO_NAMED("rrbot_hw_interface", "Connecting to the services...");

  ros::service::waitForService("/mirte/set_servoRot_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoShoulder_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoElbow_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoWrist_servo_angle", -1);
  { // Only mutex when actually writing to class vars.
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    client0 = nh_.serviceClient<mirte_msgs::SetServoAngle>(
        "/mirte/set_servoRot_servo_angle", true);
    client1 = nh_.serviceClient<mirte_msgs::SetServoAngle>(
        "/mirte/set_servoShoulder_servo_angle", true);
    client2 = nh_.serviceClient<mirte_msgs::SetServoAngle>(
        "/mirte/set_servoElbow_servo_angle", true);
    client3 = nh_.serviceClient<mirte_msgs::SetServoAngle>(
        "/mirte/set_servoWrist_servo_angle", true);
  }
  ROS_INFO_NAMED("rrbot_hw_interface", "Connected to the services");
}

void RRBotHWInterface::read(ros::Duration &elapsed_time) {
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    joint_position_[joint_id] = data[joint_id];
  }
}
void RRBotHWInterface ::start_reconnect() {
  using namespace std::chrono_literals;

  if (this->reconnect_thread.valid()) { // does it already exist or not?

    // Use wait_for() with zero milliseconds to check thread status.
    auto status = this->reconnect_thread.wait_for(0ms);

    if (status !=
        std::future_status::ready) { // Still running -> already reconnecting
      return;
    }
  }

  /* Run the reconnection on a different thread to not pause the ros-control
    loop. The launch policy std::launch::async makes sure that the task is run
    asynchronously on a new thread. */

  this->reconnect_thread =
      std::async(std::launch::async, [this] { this->connectServices(); });
}
void RRBotHWInterface::write(ros::Duration &elapsed_time) {
  // Safety
  enforceLimits(elapsed_time);

  if (initialized) {
    srv0.request.angle = (float)(joint_position_command_[0]);
    srv1.request.angle = (float)(joint_position_command_[1]);
    srv2.request.angle = (float)(joint_position_command_[2]);
    srv3.request.angle = (float)(joint_position_command_[3]);
  } else {
    if (servo_init[0] && servo_init[1] && servo_init[2] && servo_init[3]) {
      // TOOD: why do we get a segfault when we
      // set joint_positino_command_ in the contructor?
      srv0.request.angle = data[0];
      srv1.request.angle = data[1];
      srv2.request.angle = data[2];
      srv3.request.angle = data[3];
      ++init_steps;

      joint_position_command_[0] = data[0];
      joint_position_command_[1] = data[1];
      joint_position_command_[2] = data[2];
      joint_position_command_[3] = data[3];

      if (init_steps == 50) {
        initialized = true;
        ROS_INFO_NAMED("rrbot_hw_interface", "Initialized arm");
      }
    }
  }

  if (servo_init[0] && servo_init[1] && servo_init[2] && servo_init[3]) {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    if (!client0.call(srv0)) {
      ROS_INFO_NAMED("rrbot_hw_interface", "Motor 0 error");
      this->start_reconnect();
      return;
    }

    if (!client1.call(srv1)) {
      ROS_INFO_NAMED("rrbot_hw_interface", "Motor 1 error");
      this->start_reconnect();
      return;
    }

    if (!client2.call(srv2)) {
      ROS_INFO_NAMED("rrbot_hw_interface", "Motor 2 error");
      this->start_reconnect();
      return;
    }

    if (!client3.call(srv3)) {
      ROS_INFO_NAMED("rrbot_hw_interface", "Motor 3 error");
      this->start_reconnect();
      return;
    }
  }
}

void RRBotHWInterface::enforceLimits(ros::Duration &period) {
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

} // namespace rrbot_control
