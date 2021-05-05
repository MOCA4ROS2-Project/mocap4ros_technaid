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

#include <memory>

#include "mocap4ros2_technaid/MCSNode.hpp"

#include "mocap_control/ControlledLifecycleNode.hpp"
#include "ros/ros.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mocap_technaid");
  ros::NodeHandle n;

  mocap_technaid::MCSNode mcs_node;
  mcs_node.trigger_transition(mocap_control::CONFIGURE);
  ros::spin();

  mcs_node.device_cleanup();

  return 0;
}
