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

#ifndef FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP
#define FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP

#include <deque>
#include <mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <amr_v3_msgs/ErrorMode.h>

#include <amr_v3_autodocking/AutoDockingGoal.h>
#include <amr_v3_autodocking/AutoDockingAction.h>
#include <follow_waypoints/FollowWaypointsGoal.h>
#include <follow_waypoints/FollowWaypointsAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/Client.hpp>
#include <free_fleet/messages/Location.hpp>

#include "ClientNodeConfig.hpp"

namespace free_fleet
{
namespace ros1
{

class ClientNode
{
public:

  using SharedPtr = std::shared_ptr<ClientNode>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  using AutoDockClient = 
      actionlib::SimpleActionClient<amr_v3_autodocking::AutoDockingAction>;
  using AutoDockClientSharedPtr = std::shared_ptr<AutoDockClient>;

  using FollowWaypointsClient = 
      actionlib::SimpleActionClient<follow_waypoints::FollowWaypointsAction>;
  using FollowWaypointsClientSharedPtr = std::shared_ptr<FollowWaypointsClient>;

  using GoalState = actionlib::SimpleClientGoalState;

  static SharedPtr make(const ClientNodeConfig& config);

  ~ClientNode();

  struct Fields
  {
    /// Free fleet client
    Client::SharedPtr client;


    /// follow waypoints action client
    FollowWaypointsClientSharedPtr follow_waypoints_client;

    /// autodock action client
    AutoDockClientSharedPtr autodock_client;
  };

  void print_config();

private:

  // --------------------------------------------------------------------------
  // Basic ROS 1 items

  std::unique_ptr<ros::NodeHandle> node;

  std::unique_ptr<ros::Rate> update_rate;

  std::unique_ptr<ros::Rate> publish_rate;

  // --------------------------------------------------------------------------
  // Robot logic handling
    // Publisher:
  ros::Publisher cmd_runonce_pub;
  ros::Publisher cmd_brake_pub;
  // ros::Publisher cmd_cancel_pub;
  ros::Publisher mode_error_pub;

    // Subcriber:
  ros::Subscriber emergency_stop_sub;
  ros::Subscriber cmd_pause_amr_sub;
  ros::Subscriber cmd_reset_amr_sub;

  void emergency_stop_callback(const std_msgs::Bool& msg);
  void cmd_pause_amr_callback(const std_msgs::Bool& msg);
  void cmd_reset_error_amr_callback(const std_msgs::Empty& msg);
  // --------------------------------------------------------------------------
  // Battery handling

  ros::Subscriber battery_percent_sub;

  std::mutex battery_state_mutex;

  sensor_msgs::BatteryState current_battery_state;

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg);

  // --------------------------------------------------------------------------
  // Robot transform handling

  tf2_ros::Buffer tf2_buffer;

  tf2_ros::TransformListener tf2_listener;

  std::mutex robot_transform_mutex;

  geometry_msgs::TransformStamped current_robot_transform;

  geometry_msgs::TransformStamped previous_robot_transform;

  bool get_robot_transform();

  // --------------------------------------------------------------------------
  // Mode handling

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  // TODO: figure out a better way to handle multiple triggered modes
  std::atomic<bool> request_error;
  std::atomic<bool> emergency;
  std::atomic<bool> paused;
  std::atomic<bool> docking;
  std::atomic<bool> state_runonce;
  // std::atomic<bool> state_brake;


  messages::RobotMode get_robot_mode();

  bool read_mode_request();

  // --------------------------------------------------------------------------
  // Path request handling

  bool read_path_request();

  // --------------------------------------------------------------------------
  // Destination request handling

  bool read_destination_request();

  // --------------------------------------------------------------------------
  // Dock request handling

  bool read_dock_request();

  // --------------------------------------------------------------------------
  // Cancel request handling

  bool read_cancel_request();

  // --------------------------------------------------------------------------
  // Task handling

  bool is_valid_request(
      const std::string& request_fleet_name,
      const std::string& request_robot_name,
      const std::string& request_task_id);

  move_base_msgs::MoveBaseGoal location_to_move_base_goal(
      const messages::Location& location) const;

  follow_waypoints::FollowWaypointsGoal location_to_follow_waypoints_goal(
      const messages::Location& locations) const;

  follow_waypoints::FollowWaypointsGoal location_to_follow_waypoints_goal(
      const std::vector<messages::Location>& locations) const;

  amr_v3_autodocking::AutoDockingGoal location_to_autodock_goal(
      const messages::Location& locations, const messages::DockMode& _mode,
      const bool custom_docking, const int16_t _rotate_to_dock,
      const int16_t rotate_angle, const int16_t rotate_direction) const;

  std::mutex task_id_mutex;

  std::string current_task_id;

  struct DockGoal 
  {
    amr_v3_autodocking::AutoDockingGoal autodock_goal;
    bool start_docking = false;
    bool sent = false;
    uint32_t aborted_count = 0;
  };

  struct WaypointsGoal 
  {
    follow_waypoints::FollowWaypointsGoal follow_waypoints_path;
    bool sent = false;
    uint32_t aborted_count = 0;
    ros::Time goal_end_time;
  };

  struct Goal
  {
    std::string level_name;
    move_base_msgs::MoveBaseGoal goal;
    bool obey_approach_speed_limit = false;
    float approach_speed_limit;
    bool sent = false;
    uint32_t aborted_count = 0;
    ros::Time goal_end_time;
  };


  std::mutex goal_path_mutex;

  std::deque<Goal> goal_path;
  WaypointsGoal waypoints_path;
  
  std::mutex dock_goal_mutex;
  DockGoal dock_goal;

  void read_requests();

  void handle_requests();

  void publish_robot_state();

  // --------------------------------------------------------------------------
  // Custom for get feedback follow waypoints
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const follow_waypoints::FollowWaypointsResultConstPtr& result);

  void activeCb();

  void feedbackCb(const follow_waypoints::FollowWaypointsFeedbackConstPtr& feedback);

  void reset_waypoints_path();

  void reset_autodock_goal();

  void cmd_runonce(bool run);

  void cmd_brake(bool brake);

  // void cmd_cancel(bool cancel);

  void error_mode_handle(uint32_t error_mode);

  // --------------------------------------------------------------------------
  // Threads and thread functions

  std::thread update_thread;

  std::thread publish_thread;

  void update_thread_fn();

  void publish_thread_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;

  Fields fields;

  ClientNode(const ClientNodeConfig& config);

  void start(Fields fields);

};

} // namespace ros1
} // namespace free_fleet

#endif // FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP
