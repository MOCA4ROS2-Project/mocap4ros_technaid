// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <string>

#include "mocap4ros2_technaid/MCSNode.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mocap_technaid
{

MCSNode::MCSNode(const std::string & port)
: LifecycleNode("mocap_technaid"), mcs_(port)
{
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("mcs_imu", rclcpp::QoS(1000));
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MCSNode::on_configure(const rclcpp_lifecycle::State & state)
{
  if (mcs_.connect()) {
    auto info = mcs_.get_imu_info();
    mcs_technaid::print_info(info);
    return CallbackReturnT::SUCCESS;
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
MCSNode::on_activate(const rclcpp_lifecycle::State & state)
{
  mcs_.start_capture(std::bind(&MCSNode::callback_imu_data, this, std::placeholders::_1));
  imu_pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MCSNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  mcs_.stop_capture();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MCSNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  mcs_.disconnect();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MCSNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  mcs_.disconnect();
  return CallbackReturnT::SUCCESS;
}

void
MCSNode::callback_imu_data(const mcs_technaid::QuatPhyFrame* imu_data)
{
  for (int i = 0; i < imu_data->sensor_readings.size(); i++) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_" + std::to_string(i);
    imu_msg.header.stamp = now();
    imu_msg.orientation.w = imu_data->sensor_readings[i].data[0];
    imu_msg.orientation.x = imu_data->sensor_readings[i].data[1];
    imu_msg.orientation.y = imu_data->sensor_readings[i].data[2];
    imu_msg.orientation.z = imu_data->sensor_readings[i].data[3];
    imu_msg.linear_acceleration.x = imu_data->sensor_readings[i].data[4];
    imu_msg.linear_acceleration.y = imu_data->sensor_readings[i].data[5];
    imu_msg.linear_acceleration.z = imu_data->sensor_readings[i].data[6];
    imu_msg.angular_velocity.x = imu_data->sensor_readings[i].data[7];
    imu_msg.angular_velocity.y = imu_data->sensor_readings[i].data[8];
    imu_msg.angular_velocity.z = imu_data->sensor_readings[i].data[9];
    imu_pub_->publish(imu_msg);
  }
}

void
MCSNode::device_cleanup()
{
  if (mcs_.is_capturing()) {
    mcs_.stop_capture();
  }

  if (mcs_.is_connected()) {
    mcs_.disconnect();
  }
}

}  // mocap_technaid

