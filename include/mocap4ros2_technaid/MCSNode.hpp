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

#ifndef MOCAP4ROS2_TECHNAID__MCSNODE_HPP__
#define MOCAP4ROS2_TECHNAID__MCSNODE_HPP__

#include <string>
#include <vector>

#include "mcs_technaid/MCS.hpp"

#include "sensor_msgs/Imu.h"
#include "mocap_msgs/ImusInfo.h"

#include "mocap_control/ControlledLifecycleNode.hpp"

#include "ros/ros.h"

namespace mocap_technaid
{

class MCSNode : public mocap_control::ControlledLifecycleNode
{
public:
  explicit MCSNode(const std::string & port = "automatic");

  bool on_configure();
  bool on_activate();
  bool on_deactivate();

  void device_cleanup();
private:
  void callback_imu_data(const mcs_technaid::QuatPhyFrame * orien_phy_frame);

  mcs_technaid::MCS mcs_;
  mcs_technaid::MCSInfo mcs_info_;

  ros::Publisher imu_pub_;
  std::vector<ros::Publisher> imu_individual_pubs_;
  ros::Publisher imus_info_pub_;
};

}  // namespace mocap_technaid


#endif  // MOCAP4ROS2_TECHNAID__MCSNODE_HPP__
