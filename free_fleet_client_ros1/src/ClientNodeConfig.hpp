/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODECONFIG_HPP
#define FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODECONFIG_HPP

#include <string>

#include <ros/ros.h>

#include <free_fleet/ClientConfig.hpp>

namespace free_fleet
{
namespace ros1
{

struct ClientNodeConfig
{

  std::string fleet_name = "fleet_name";
  std::string robot_name = "robot_name";
  std::string robot_model = "robot_model";
  std::string level_name = "level_name";

  std::string cmd_runonce_topic = "state_runonce_nav";
//   std::string cmd_cancel_topic = "CANCEL_AMR_FROM_SERVER";
  std::string cmd_pause_topic = "PAUSE_AMR";
  std::string cmd_reset_topic = "RESET_AMR";
  std::string cmd_breaker_topic = "cmd_brake";
  std::string mode_error_topic = "error_mode";
  std::string emergency_stop_topic = "emergency_stop";

  std::string battery_state_topic = "/battery_state";

  std::string map_frame = "map";
  std::string robot_frame = "base_footprint";

  std::string follow_waypoints_server_name = "follow_waypoints_server";
  std::string autodock_server_name = "autodock";


  int dds_domain = 42;
  std::string dds_state_topic = "robot_state";
  std::string dds_mode_request_topic = "mode_request";
  std::string dds_path_request_topic = "path_request";
  std::string dds_destination_request_topic = "destination_request";
  std::string dds_dock_request_topic = "dock_request";
  std::string dds_cancel_request_topic = "cancel_request";

  double wait_timeout = 10.0;
  double update_frequency = 10.0;
  double publish_frequency = 1.0;

  double max_dist_to_first_waypoint = 10.0;

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key, 
      std::string& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      int& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      double& param_out);

  void print_config() const;

  ClientConfig get_client_config() const;

  static ClientNodeConfig make();

};

} // namespace ros1
} // namespace free_fleet

#endif // FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODECONFIG_HPP
