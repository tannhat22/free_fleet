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

#include <cstdio>

#include "ClientNodeConfig.hpp"

namespace free_fleet
{
namespace ros1
{

void ClientNodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    std::string& _param_out)
{
  std::string tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %s", 
        _key.c_str(), _key.c_str(), tmp_param.c_str());
    _param_out = tmp_param;
  }
}

void ClientNodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    int& _param_out)
{
  int tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %d.",
        _key.c_str(), _key.c_str(), tmp_param);
    _param_out = tmp_param;
  }
}

void ClientNodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    double& _param_out)
{
  double tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %.2f.",
        _key.c_str(), _key.c_str(), tmp_param);
    _param_out = tmp_param;
  }
}

void ClientNodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    bool& _param_out)
{
  bool tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %d.",
        _key.c_str(), _key.c_str(), tmp_param);
    _param_out = tmp_param;
  }
}

void ClientNodeConfig::print_config() const
{
  printf("ROS 1 CLIENT CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  robot name: %s\n", robot_name.c_str());
  printf("  robot model: %s\n", robot_model.c_str());
  printf("  level name: %s\n", level_name.c_str());
  printf("  wait for intialpose: %s\n", wait_for_intialpose ? "true" : "false");
  printf("  wait timeout: %.1f\n", wait_timeout);
  printf("  wait timeout intialpose: %.1f\n", wait_timeout_intialpose);
  printf("  update request frequency: %.1f\n", update_frequency);
  printf("  publish state frequency: %.1f\n", publish_frequency);
  printf("  maximum distance to first waypoint: %.1f\n", 
      max_dist_to_first_waypoint);
  printf("  TOPICS\n");
  printf("    cmd_runonce: %s\n", cmd_runonce_topic.c_str());
//   printf("    cmd_cancel: %s\n", cmd_cancel_topic.c_str());
  printf("    cmd_pause: %s\n", cmd_pause_topic.c_str());
  printf("    cmd_reset: %s\n", cmd_reset_topic.c_str());
  printf("    cmd_breaker: %s\n", cmd_breaker_topic.c_str());
  printf("    mode_error: %s\n", mode_error_topic.c_str());
  printf("    emergency stop: %s\n", emergency_stop_topic.c_str());
  printf("    battery state: %s\n", battery_state_topic.c_str());
  printf("    is_intialpose: %s\n", is_intialpose_topic.c_str());
  printf("    follow waypoints server: %s\n", follow_waypoints_server_name.c_str());
  printf("    autodock server: %s\n", autodock_server_name.c_str());
  printf("    localize server: %s\n", localize_server_name.c_str());
  printf("  ROBOT FRAMES\n");
  printf("    map frame: %s\n", map_frame.c_str());
  printf("    robot frame: %s\n", robot_frame.c_str());
  printf("CLIENT-SERVER DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", dds_state_topic.c_str());
  printf("    mode request: %s\n", dds_mode_request_topic.c_str());
  printf("    path request: %s\n", dds_path_request_topic.c_str());
  printf("    destination request: %s\n", dds_destination_request_topic.c_str());
  printf("    dock request: %s\n", dds_dock_request_topic.c_str());
  printf("    cancel request: %s\n", dds_cancel_request_topic.c_str());
  printf("    localize request: %s\n", dds_localize_request_topic.c_str());
}
  
ClientConfig ClientNodeConfig::get_client_config() const
{
  ClientConfig client_config;
  client_config.dds_domain = dds_domain;
  client_config.dds_state_topic = dds_state_topic;
  client_config.dds_mode_request_topic = dds_mode_request_topic;
  client_config.dds_path_request_topic = dds_path_request_topic;
  client_config.dds_destination_request_topic = dds_destination_request_topic;
  client_config.dds_dock_request_topic = dds_dock_request_topic;
  client_config.dds_cancel_request_topic = dds_cancel_request_topic;
  client_config.dds_localize_request_topic = dds_localize_request_topic;
  return client_config;
}

ClientNodeConfig ClientNodeConfig::make()
{
  ClientNodeConfig config;
  ros::NodeHandle node_private_ns("~");
  config.get_param_if_available(
      node_private_ns, "fleet_name", config.fleet_name);
  config.get_param_if_available(
      node_private_ns, "robot_name", config.robot_name);
  config.get_param_if_available(
      node_private_ns, "robot_model", config.robot_model);
  config.get_param_if_available(
      node_private_ns, "level_name", config.level_name);
  config.get_param_if_available(
      node_private_ns, "cmd_runonce_topic", config.cmd_runonce_topic);
//   config.get_param_if_available(
//       node_private_ns, "cmd_cancel_topic", config.cmd_cancel_topic);
  config.get_param_if_available(
      node_private_ns, "cmd_pause_topic", config.cmd_pause_topic);
  config.get_param_if_available(
      node_private_ns, "cmd_reset_topic", config.cmd_reset_topic);
  config.get_param_if_available(
      node_private_ns, "cmd_breaker_topic", config.cmd_breaker_topic);
  config.get_param_if_available(
      node_private_ns, "mode_error_topic", config.mode_error_topic);
  config.get_param_if_available(
      node_private_ns, "emergency_stop_topic", config.emergency_stop_topic);
  config.get_param_if_available(
      node_private_ns, "battery_state_topic", config.battery_state_topic);
  config.get_param_if_available(
      node_private_ns, "is_intialpose_topic", config.is_intialpose_topic);
  config.get_param_if_available(
      node_private_ns, "map_frame", config.map_frame);
  config.get_param_if_available(
      node_private_ns, "robot_frame", config.robot_frame);
  config.get_param_if_available(
      node_private_ns, "follow_waypoints_server_name", config.follow_waypoints_server_name);
  config.get_param_if_available(
      node_private_ns, "autodock_server_name", config.autodock_server_name);
  config.get_param_if_available(
      node_private_ns, "localize_server_name", config.localize_server_name);
  config.get_param_if_available(
      node_private_ns, "dds_domain", config.dds_domain);
  config.get_param_if_available(
      node_private_ns, "dds_mode_request_topic", config.dds_mode_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_path_request_topic", config.dds_path_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_destination_request_topic", config.dds_destination_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_dock_request_topic", config.dds_dock_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_cancel_request_topic", config.dds_cancel_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_localize_request_topic", config.dds_localize_request_topic);
  config.get_param_if_available(
      node_private_ns, "wait_for_intialpose", config.wait_for_intialpose);
  config.get_param_if_available(
      node_private_ns, "wait_timeout", config.wait_timeout);
  config.get_param_if_available(
      node_private_ns, "wait_timeout_intialpose", config.wait_timeout_intialpose);
  config.get_param_if_available(
      node_private_ns, "update_frequency", config.update_frequency);
  config.get_param_if_available(
      node_private_ns, "publish_frequency", config.publish_frequency);
  config.get_param_if_available(
      node_private_ns, "max_dist_to_first_waypoint", 
      config.max_dist_to_first_waypoint);
  return config;
}

} // namespace ros1
} // namespace free_fleet
