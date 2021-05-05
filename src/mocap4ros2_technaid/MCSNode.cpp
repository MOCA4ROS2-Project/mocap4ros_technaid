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

#include "sensor_msgs/Imu.h"
#include "mocap_msgs/ImusInfo.h"

#include "ros/ros.h"

namespace mocap_technaid
{

MCSNode::MCSNode(const std::string & port)
: ControlledLifecycleNode(ros::this_node::getName()), mcs_(port)
{
  imu_pub_ = create_publisher<sensor_msgs::Imu>("mopcap_imu_data", 1000);
  imus_info_pub_ = create_publisher<mocap_msgs::ImusInfo>("mocap_imu_info", 10, true);
}

bool
MCSNode::on_configure()
{
  if (mcs_.connect()) {
    mcs_info_ = mcs_.get_imu_info();
    mcs_technaid::print_info(mcs_info_);

    imu_individual_pubs_.clear();
    for (int i = 0; i < mcs_info_.sensor_ids.size(); i++) {
      std::string id = "imu_" + std::to_string(mcs_info_.sensor_ids[i]);
      imu_individual_pubs_.push_back(
        create_publisher<sensor_msgs::Imu>(
          "mopcap_imu_data/" + id,
          1000));
    }

    return ControlledLifecycleNode::on_configure();
  } else {
    return false;
  }
}

bool
MCSNode::on_activate()
{
  mcs_.start_capture(std::bind(&MCSNode::callback_imu_data, this, std::placeholders::_1));

  mocap_msgs::ImusInfo info_msg;
  for (const auto & sensor : mcs_info_.sensor_ids) {
    info_msg.sensor_ids.push_back(std::to_string(sensor));
  }

  info_msg.battery_level = mcs_info_.battery_level;
  info_msg.temperature = mcs_info_.temperature;

  imus_info_pub_.publish(info_msg);

  return ControlledLifecycleNode::on_activate();
}

bool
MCSNode::on_deactivate()
{
  mcs_.stop_capture();
  return ControlledLifecycleNode::on_deactivate();
}

void
MCSNode::callback_imu_data(const mcs_technaid::QuatPhyFrame * imu_data)
{
  for (int i = 0; i < imu_data->sensor_readings.size(); i++) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu_" + std::to_string(mcs_info_.sensor_ids[i]);
    imu_msg.header.stamp = ros::Time::now();
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

    if (imu_pub_.getNumSubscribers() > 0) {
      imu_pub_.publish(imu_msg);
    }

    if (imu_individual_pubs_[i].getNumSubscribers() > 0) {
      imu_individual_pubs_[i].publish(imu_msg);
    }
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

}  // namespace mocap_technaid

