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

#include "utilities.hpp"
#include "ClientNode.hpp"
#include "ClientNodeConfig.hpp"

namespace free_fleet
{
namespace ros1
{

ClientNode::SharedPtr ClientNode::make(const ClientNodeConfig& _config)
{
  SharedPtr client_node = SharedPtr(new ClientNode(_config));
  client_node->node.reset(new ros::NodeHandle(_config.robot_name + "_node"));

  /// Starting the free fleet client
  ClientConfig client_config = _config.get_client_config();
  Client::SharedPtr client = Client::make(client_config);
  if (!client)
    return nullptr;


  /// Setting up the follow waypoints action client, wait for server
  ROS_INFO("waiting for connection with follow waypoints action server: %s",
      _config.follow_waypoints_server_name.c_str());
  FollowWaypointsClientSharedPtr follow_waypoints_client(
      new FollowWaypointsClient(_config.follow_waypoints_server_name, true));
  if (!follow_waypoints_client->waitForServer(ros::Duration(_config.wait_timeout)))
  {
    ROS_ERROR("timed out waiting for action server: %s",
        _config.follow_waypoints_server_name.c_str());
    return nullptr;
  }
  ROS_INFO("connected with follow waypoints action server: %s",
      _config.follow_waypoints_server_name.c_str());
  /////////////////////////////////////////////////////////////

  /// Setting up the autodock action client, wait for server
  ROS_INFO("waiting for connection with autodock action server: %s",
      _config.autodock_server_name.c_str());
  AutoDockClientSharedPtr autodock_client(
      new AutoDockClient(_config.autodock_server_name, true));
  if (!autodock_client->waitForServer(ros::Duration(_config.wait_timeout)))
  {
    ROS_ERROR("timed out waiting for action server: %s",
        _config.autodock_server_name.c_str());
    return nullptr;
  }
  ROS_INFO("connected with autodock action server: %s",
      _config.autodock_server_name.c_str());
  /////////////////////////////////////////////////////////////

  client_node->start(Fields{
      std::move(client),
      std::move(follow_waypoints_client),
      std::move(autodock_client),
  });

  return client_node;
}

ClientNode::ClientNode(const ClientNodeConfig& _config) :
  tf2_listener(tf2_buffer),
  client_node_config(_config)
{}

ClientNode::~ClientNode()
{
  if (update_thread.joinable())
  {
    update_thread.join();
    ROS_INFO("Client: update_thread joined.");
  }

  if (publish_thread.joinable())
  {
    publish_thread.join();
    ROS_INFO("Client: publish_thread joined.");
  }
}

void ClientNode::start(Fields _fields)
{
  fields = std::move(_fields);

  update_rate.reset(new ros::Rate(client_node_config.update_frequency));
  publish_rate.reset(new ros::Rate(client_node_config.publish_frequency));

  request_error = false;
  emergency = false;
  paused = false;
  docking = false;
  state_runonce = false;  

  // Runonce pub
  cmd_runonce_pub = node->advertise<std_msgs::Bool>(
    client_node_config.cmd_runonce_topic, 10);

  // Brake pub
  cmd_brake_pub = node->advertise<std_msgs::Bool>(
    client_node_config.cmd_breaker_topic, 10);

  // Cancel pub
  // cmd_cancel_pub = node->advertise<std_msgs::Empty>(
  //   client_node_config.cmd_cancel_topic, 10);

  // Error mode pub
  mode_error_pub = node->advertise<amr_v3_msgs::ErrorStamped>(
    client_node_config.mode_error_topic, 10);

  // Emergency stop sub
  emergency_stop_sub = node->subscribe(
      client_node_config.emergency_stop_topic, 1,
      &ClientNode::emergency_stop_callback, this);

  // Pause sub
  cmd_pause_amr_sub = node->subscribe(
      client_node_config.cmd_pause_topic, 1,
      &ClientNode::cmd_pause_amr_callback, this);

  // Reset error sub
  cmd_reset_amr_sub = node->subscribe(
      client_node_config.cmd_reset_topic, 1,
      &ClientNode::cmd_reset_error_amr_callback, this);

  // Battery state sub 
  battery_percent_sub = node->subscribe(
      client_node_config.battery_state_topic, 1,
      &ClientNode::battery_state_callback_fn, this);

  ROS_INFO("Client: starting update thread.");
  update_thread = std::thread(std::bind(&ClientNode::update_thread_fn, this));

  ROS_INFO("Client: starting publish thread.");
  publish_thread = 
      std::thread(std::bind(&ClientNode::publish_thread_fn, this));
}

void ClientNode::print_config()
{
  client_node_config.print_config();
}

void ClientNode::emergency_stop_callback(
  const std_msgs::Bool& _msg)
{
  emergency = _msg.data;
  if (emergency) {
    fields.follow_waypoints_client->cancelAllGoals();
    WriteLock goal_path_lock(goal_path_mutex);
    goal_path.clear();
    reset_waypoints_path();

    fields.autodock_client->cancelAllGoals();
    WriteLock dock_goal_lock(dock_goal_mutex);
    reset_autodock_goal();

    if (state_runonce) {
      cmd_runonce(false);
      state_runonce = false;
    }  

    if (paused) {
      paused = false;
    }
  }
}

void ClientNode::cmd_pause_amr_callback(
    const std_msgs::Bool& _msg)
{
  if (paused != _msg.data) {
    if (_msg.data) {
      fields.follow_waypoints_client->cancelAllGoals();
      WriteLock goal_path_lock(goal_path_mutex);
      if (!goal_path.empty()) {
        goal_path[0].sent = false;
        waypoints_path.sent = false;
      }
    }
    paused = _msg.data;
    // emergency = false;
  }
}

void ClientNode::cmd_reset_error_amr_callback(const std_msgs::Empty& _msg)
{
  request_error = false;
}

void ClientNode::battery_state_callback_fn(
    const sensor_msgs::BatteryState& _msg)
{
  WriteLock battery_state_lock(battery_state_mutex);
  current_battery_state = _msg;
}

bool ClientNode::get_robot_transform()
{
  try {
    geometry_msgs::TransformStamped tmp_transform_stamped = 
        tf2_buffer.lookupTransform(
            client_node_config.map_frame,
            client_node_config.robot_frame,
            ros::Time(0));
    WriteLock robot_transform_lock(robot_transform_mutex);
    previous_robot_transform = current_robot_transform;
    current_robot_transform = tmp_transform_stamped;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}

messages::RobotMode ClientNode::get_robot_mode()
{
  /// Checks if robot has just received a request that causes an adapter error
  if (request_error)
    return messages::RobotMode{messages::RobotMode::MODE_REQUEST_ERROR};

  /// Checks if robot is under emergency
  if (emergency)
    return messages::RobotMode{messages::RobotMode::MODE_EMERGENCY};

  /// Checks if robot is docking:
  if (docking)
    return messages::RobotMode{messages::RobotMode::MODE_DOCKING};

  /// Checks if robot is charging
  {
    ReadLock battery_state_lock(battery_state_mutex);

    if (current_battery_state.power_supply_status ==
        current_battery_state.POWER_SUPPLY_STATUS_CHARGING)
      return messages::RobotMode{messages::RobotMode::MODE_CHARGING};
  }  

  /// Checks if robot is moving
  {
    ReadLock robot_transform_lock(robot_transform_mutex);

    if (!is_transform_close(
        current_robot_transform, previous_robot_transform))
      return messages::RobotMode{messages::RobotMode::MODE_MOVING};
  }
  
  /// Otherwise, robot is neither charging nor moving,
  /// Checks if the robot is paused
  if (paused)
    return messages::RobotMode{messages::RobotMode::MODE_PAUSED};

  /// Otherwise, robot has queued tasks, it is paused or waiting,
  /// default to use pausing for now
  return messages::RobotMode{messages::RobotMode::MODE_IDLE};
}

void ClientNode::publish_robot_state()
{
  messages::RobotState new_robot_state;
  new_robot_state.name = client_node_config.robot_name;
  new_robot_state.model = client_node_config.robot_model;

  {
    ReadLock task_id_lock(task_id_mutex);
    new_robot_state.task_id = current_task_id;    
  }

  new_robot_state.mode = get_robot_mode();

  {
    ReadLock battery_state_lock(battery_state_mutex);
    /// RMF expects battery to have a percentage in the range for 0-100.
    /// sensor_msgs/BatteryInfo on the other hand returns a value in 
    /// the range of 0-1
    new_robot_state.battery_percent = 100*current_battery_state.percentage;
  }

  {
    ReadLock robot_transform_lock(robot_transform_mutex);
    new_robot_state.location.sec = current_robot_transform.header.stamp.sec;
    new_robot_state.location.nanosec = 
        current_robot_transform.header.stamp.nsec;
    new_robot_state.location.x = 
        current_robot_transform.transform.translation.x;
    new_robot_state.location.y = 
        current_robot_transform.transform.translation.y;
    new_robot_state.location.yaw = 
        get_yaw_from_transform(current_robot_transform);
    new_robot_state.location.obey_approach_speed_limit = false;
    new_robot_state.location.approach_speed_limit = 0.0;
    new_robot_state.location.level_name = client_node_config.level_name;
  }

  new_robot_state.path.clear();
  {
    ReadLock goal_path_lock(goal_path_mutex);

    for (size_t i = 0; i < goal_path.size(); ++i)
    {
      new_robot_state.path.push_back(
          messages::Location{
              (int32_t)goal_path[i].goal.target_pose.header.stamp.sec,
              goal_path[i].goal.target_pose.header.stamp.nsec,
              (float)goal_path[i].goal.target_pose.pose.position.x,
              (float)goal_path[i].goal.target_pose.pose.position.y,
              (float)(get_yaw_from_quat(
                  goal_path[i].goal.target_pose.pose.orientation)),
              (bool)goal_path[i].obey_approach_speed_limit,
              (float)goal_path[i].approach_speed_limit,
              goal_path[i].level_name
          });
    }
  }

  if (!fields.client->send_robot_state(new_robot_state))
    ROS_WARN("failed to send robot state: msg sec %u", new_robot_state.location.sec);
}

bool ClientNode::is_valid_request(
    const std::string& _request_fleet_name,
    const std::string& _request_robot_name,
    const std::string& _request_task_id)
{
  ReadLock task_id_lock(task_id_mutex);
  if (current_task_id == _request_task_id ||
      client_node_config.robot_name != _request_robot_name ||
      client_node_config.fleet_name != _request_fleet_name)
    return false;
  return true;
}

move_base_msgs::MoveBaseGoal ClientNode::location_to_move_base_goal(
    const messages::Location& _location) const
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = client_node_config.map_frame;
  goal.target_pose.header.stamp.sec = _location.sec;
  goal.target_pose.header.stamp.nsec = _location.nanosec;
  goal.target_pose.pose.position.x = _location.x;
  goal.target_pose.pose.position.y = _location.y;
  goal.target_pose.pose.position.z = 0.0; // TODO: handle Z height with level
  goal.target_pose.pose.orientation = get_quat_from_yaw(_location.yaw);
  return goal;
}

follow_waypoints::FollowWaypointsGoal ClientNode::location_to_follow_waypoints_goal(
    const messages::Location& _locations) const
{
  follow_waypoints::FollowWaypointsGoal goal;
  goal.target_poses.header.frame_id = client_node_config.map_frame;
  goal.target_poses.header.stamp.sec = _locations.sec;
  goal.target_poses.header.stamp.nsec = _locations.nanosec;
  geometry_msgs::Pose pose;
  pose.position.x = _locations.x;
  pose.position.y = _locations.y;
  pose.position.z = 0.0; 
  pose.orientation = get_quat_from_yaw(_locations.yaw);
  goal.target_poses.poses.push_back(pose);
  goal.obey_approach_speed_limit.push_back(_locations.obey_approach_speed_limit);
  goal.approach_speed_limit.push_back(_locations.approach_speed_limit);
  return goal;
}

follow_waypoints::FollowWaypointsGoal ClientNode::location_to_follow_waypoints_goal(
    const std::vector<messages::Location>& _locations) const
{
  follow_waypoints::FollowWaypointsGoal goal;
  goal.target_poses.header.frame_id = client_node_config.map_frame;
  goal.target_poses.header.stamp.sec = _locations[0].sec;
  goal.target_poses.header.stamp.nsec = _locations[0].nanosec;
  for (const auto& location : _locations)
  {
    geometry_msgs::Pose pose;
    pose.position.x = location.x;
    pose.position.y = location.y;
    pose.position.z = 0.0; 
    pose.orientation = get_quat_from_yaw(location.yaw);
    goal.target_poses.poses.push_back(pose);
    goal.obey_approach_speed_limit.push_back(location.obey_approach_speed_limit);
    goal.approach_speed_limit.push_back(location.approach_speed_limit);
  }
  return goal;
}

amr_v3_autodocking::AutoDockingGoal ClientNode::location_to_autodock_goal(
      const messages::Location& _location, const messages::DockMode& _mode,
      const bool _custom_docking, const int16_t _rotate_to_dock,
      const int16_t _rotate_angle, const int16_t _rotate_direction ) const
{
  amr_v3_autodocking::AutoDockingGoal goal;
  goal.dock_pose.header.frame_id = client_node_config.map_frame;
  goal.dock_pose.header.stamp.sec = _location.sec;
  goal.dock_pose.header.stamp.nsec = _location.nanosec;
  goal.dock_pose.pose.position.x = _location.x;
  goal.dock_pose.pose.position.y = _location.y;
  goal.dock_pose.pose.position.z = 0.0;
  goal.dock_pose.pose.orientation = get_quat_from_yaw(_location.yaw);
  goal.mode = _mode.mode;
  goal.custom_docking = _custom_docking;
  goal.rotate_to_dock = _rotate_to_dock;
  goal.ROTATE_ANGLE = _rotate_angle;
  goal.ROTATE_ORIENTATION = _rotate_direction;
  return goal;
}

bool ClientNode::read_mode_request()
{
  messages::ModeRequest mode_request;
  if (fields.client->read_mode_request(mode_request) && 
      is_valid_request(
          mode_request.fleet_name, mode_request.robot_name, 
          mode_request.task_id))
  {
    if (mode_request.mode.mode == messages::RobotMode::MODE_PAUSED)
    {
      ROS_INFO("received a PAUSE command.");

      fields.follow_waypoints_client->cancelAllGoals();
      WriteLock goal_path_lock(goal_path_mutex);
      if (!goal_path.empty()) {
        goal_path[0].sent = false;
        waypoints_path.sent = false;
      }

      paused = true;
      // emergency = false;
    }
    else if (mode_request.mode.mode == messages::RobotMode::MODE_MOVING)
    {
      ROS_INFO("received an explicit RESUME command.");
      paused = false;
      // emergency = false;
    }
    else if (mode_request.mode.mode == messages::RobotMode::MODE_EMERGENCY)
    {
      ROS_INFO("received an EMERGENCY command.");
      paused = false;
      emergency = true;
    }

    WriteLock task_id_lock(task_id_mutex);
    current_task_id = mode_request.task_id;

    // request_error = false;
    return true;
  }
  return false;
}

bool ClientNode::read_path_request()
{
  messages::PathRequest path_request;
  if (fields.client->read_path_request(path_request) &&
      is_valid_request(
          path_request.fleet_name, path_request.robot_name,
          path_request.task_id))
  {
    ROS_INFO("received a Path command of size %lu.", path_request.path.size());

    if (path_request.path.size() <= 0)
      return false;

    // Sanity check: the first waypoint of the Path must be within N meters of
    // our current position. Otherwise, ignore the request.
    {
      ReadLock robot_transform_lock(robot_transform_mutex);
      const double dx =
          path_request.path[0].x - 
          current_robot_transform.transform.translation.x;
      const double dy =
          path_request.path[0].y -
          current_robot_transform.transform.translation.y;
      const double dist_to_first_waypoint = sqrt(dx*dx + dy*dy);

      ROS_INFO("distance to first waypoint: %.2f\n", dist_to_first_waypoint);

      if (dist_to_first_waypoint > 
          client_node_config.max_dist_to_first_waypoint)
      {
        ROS_WARN("distance was over threshold of %.2f ! Rejecting path,"
            "waiting for next valid request.\n",
            client_node_config.max_dist_to_first_waypoint);
        
        fields.follow_waypoints_client->cancelAllGoals();
        WriteLock goal_path_lock(goal_path_mutex);
        goal_path.clear();
        reset_waypoints_path();

        request_error = true;
        error_mode_handle(amr_v3_msgs::Error::NAVIGATION_ERROR, NAV_ERROR, false);
        // emergency = false;
        paused = false;
        return false;
      }
    }

    WriteLock goal_path_lock(goal_path_mutex);
    int32_t waypoint_sec = 0;
    uint32_t waypoint_nanosec = 0;
    goal_path.clear();
    for (size_t i = 0; i < path_request.path.size(); ++i)
    {
      waypoint_sec = path_request.path[i].sec;
      waypoint_nanosec = path_request.path[i].nanosec;

      goal_path.push_back(
          Goal {
              path_request.path[i].level_name,
              location_to_move_base_goal(path_request.path[i]),
              path_request.path[i].obey_approach_speed_limit,
              path_request.path[i].approach_speed_limit,
              false,
              0,
              ros::Time(
                  path_request.path[i].sec, path_request.path[i].nanosec)});
    }
    // Get waypoint path
    waypoints_path.follow_waypoints_path = location_to_follow_waypoints_goal(path_request.path);
    waypoints_path.sent = false;
    waypoints_path.aborted_count = 0;
    waypoints_path.goal_end_time = ros::Time(waypoint_sec, waypoint_nanosec);

    WriteLock task_id_lock(task_id_mutex);
    current_task_id = path_request.task_id;

    if (paused)
      paused = false;

    // request_error = false;
    return true;
  }
  return false;
}

bool ClientNode::read_destination_request()
{
  messages::DestinationRequest destination_request;
  if (fields.client->read_destination_request(destination_request) &&
      is_valid_request(
          destination_request.fleet_name, destination_request.robot_name,
          destination_request.task_id))
  {
    ROS_INFO("received a Destination command, x: %.2f, y: %.2f, yaw: %.2f",
        destination_request.destination.x, destination_request.destination.y,
        destination_request.destination.yaw);
    
    WriteLock goal_path_lock(goal_path_mutex);
    goal_path.clear();
    reset_waypoints_path();
    goal_path.push_back(
        Goal {
            destination_request.destination.level_name,
            location_to_move_base_goal(destination_request.destination),
            destination_request.destination.obey_approach_speed_limit,
            destination_request.destination.approach_speed_limit,
            false,
            0,
            ros::Time(
                destination_request.destination.sec, 
                destination_request.destination.nanosec)});
    // Get waypoint path
    waypoints_path.follow_waypoints_path = location_to_follow_waypoints_goal(destination_request.destination);
    waypoints_path.sent = false;
    waypoints_path.aborted_count = 0;
    waypoints_path.goal_end_time = ros::Time(destination_request.destination.sec,
                                             destination_request.destination.nanosec);

    WriteLock task_id_lock(task_id_mutex);
    current_task_id = destination_request.task_id;

    if (paused)
      paused = false;

    // request_error = false;
    return true;
  }
  return false;
}

bool ClientNode::read_dock_request()
{
  messages::DockRequest dock_request;
  if (fields.client->read_dock_request(dock_request) &&
      is_valid_request(
          dock_request.fleet_name, dock_request.robot_name,
          dock_request.task_id))
  {
    if (dock_request.dock_mode.mode == messages::DockMode::MODE_CHARGE)
    {
      ROS_INFO("received Dock command, mode: CHARGE");
    }
    else if (dock_request.dock_mode.mode == messages::DockMode::MODE_PICKUP)
    {
      ROS_INFO("received Dock command, mode: PICKUP");
    }
    else if (dock_request.dock_mode.mode == messages::DockMode::MODE_DROPOFF)
    {
      ROS_INFO("received Dock command, mode: DROPOFF");
    }
    else if (dock_request.dock_mode.mode == messages::DockMode::MODE_UNDOCK)
    {
      ROS_INFO("received Dock command, mode: UNDOCK");
    }
    else {
      ROS_ERROR("received Dock command but mode does not support. Please check again!");
    }

    WriteLock dock_goal_lock(dock_goal_mutex);
    dock_goal.autodock_goal = location_to_autodock_goal(dock_request.destination,
                                                        dock_request.dock_mode,
                                                        dock_request.custom_docking,
                                                        dock_request.rotate_to_dock,
                                                        dock_request.rotate_angle,
                                                        dock_request.rotate_orientation);
    dock_goal.aborted_count = 0;
    dock_goal.sent = false;
    dock_goal.start_docking = true;

    WriteLock task_id_lock(task_id_mutex);
    current_task_id = dock_request.task_id;

    if (paused)
      paused = false;

    // request_error = false;
    return true;
  }
  return false;
}

bool ClientNode::read_cancel_request()
{
  messages::CancelRequest cancel_request;
  if (fields.client->read_cancel_request(cancel_request) &&
      is_valid_request(
          cancel_request.fleet_name, cancel_request.robot_name,
          cancel_request.task_id))
  {
    ROS_INFO("received Cancel command from Server");
    // cmd_cancel(true);
    
    fields.follow_waypoints_client->cancelAllGoals();
    WriteLock goal_path_lock(goal_path_mutex);
    goal_path.clear();
    reset_waypoints_path();

    fields.autodock_client->cancelAllGoals();
    WriteLock dock_goal_lock(dock_goal_mutex);
    reset_autodock_goal();

    WriteLock task_id_lock(task_id_mutex);
    current_task_id = cancel_request.task_id;

    if (state_runonce) {
      cmd_runonce(false);
      state_runonce = false;
    }

    if (paused)
      paused = false;

    // request_error = false;
    return true;
  }
  return false;
}

void ClientNode::read_requests()
{
  if (read_mode_request() || 
      read_path_request() || 
      read_destination_request() || 
      read_dock_request() ||
      read_cancel_request())
    return;
}

void ClientNode::handle_requests()
{
  // there is an emergency or the robot is paused
  if (emergency || request_error || paused)
    return;

  // ooooh we have goals
  WriteLock goal_path_lock(goal_path_mutex);
  if (!goal_path.empty())
  {
    if (waypoints_path.follow_waypoints_path.target_poses.poses.size() >= 1) {
      if (!waypoints_path.sent)
      {
        if (!state_runonce) {
          cmd_runonce(true);
          cmd_brake(false);
          state_runonce = true;
        }
        ROS_INFO("sending waypoints goal.");
        fields.follow_waypoints_client->sendGoal(waypoints_path.follow_waypoints_path,
                                                 boost::bind(&ClientNode::doneCb, this, _1, _2),
                                                 boost::bind(&ClientNode::activeCb, this),
                                                 boost::bind(&ClientNode::feedbackCb, this, _1));
        // goal_path.front().sent = true;
        waypoints_path.sent = true;
        return;
      }

      // Goals have been sent, check the goal states now
      GoalState current_goal_state = fields.follow_waypoints_client->getState();
      if (current_goal_state == GoalState::SUCCEEDED)
      {
        ROS_INFO("Waypoints goal state: SUCCEEEDED.");
        goal_path.clear();
        reset_waypoints_path();
        if (state_runonce) {
          cmd_runonce(false);
          state_runonce = false;
        }      
        return;
        // By some stroke of good fortune, we may have arrived at our goal
        // earlier than we were scheduled to reach it. If that is the case,
        // we need to wait here until it's time to proceed.
        // if (ros::Time::now() >= goal_path.front().goal_end_time)
        // {
        //   goal_path.clear();
        //   reset_waypoints_path();
        // }
        // else
        // {
        //   ros::Duration wait_time_remaining =
        //       waypoints_path.goal_end_time - ros::Time::now();
        //   ROS_INFO(
        //       "we reached our goal early! Waiting %.1f more seconds",
        //       wait_time_remaining.toSec());
        // }
        // return;
      }
      else if (current_goal_state == GoalState::ACTIVE)
      {
        return;
      }
      else if (current_goal_state == GoalState::ABORTED)
      {
        waypoints_path.aborted_count++;

        // TODO: parameterize the maximum number of retries.
        if (waypoints_path.aborted_count < 5)
        {
          ROS_INFO("robot's navigation stack has aborted the current goal %d "
              "times, client will trying again...",
              waypoints_path.aborted_count);
          fields.follow_waypoints_client->cancelGoal();
          // goal_path.front().sent = false;
          waypoints_path.sent = false;
          return;
        }
        else
        {
          ROS_INFO("robot's navigation stack has aborted the current goal %d "
              "times, please check that there is nothing in the way of the "
              "robot, client will abort the current path request, and await "
              "further requests.",
              waypoints_path.aborted_count);
          fields.follow_waypoints_client->cancelGoal();
          goal_path.clear();
          reset_waypoints_path();
          request_error = true;
          error_mode_handle(amr_v3_msgs::Error::NAVIGATION_ERROR, NAV_ERROR, false);
          if (state_runonce) {
            cmd_runonce(false);
            cmd_brake(true);
            state_runonce = false;
          }  
          return;
        }
      }
      else if (current_goal_state == GoalState::PREEMPTED) {
        ROS_INFO("Client was cancel the current path request, and await further "
            "requests or manual intervention.");
        goal_path.clear();
        reset_waypoints_path();
        if (state_runonce) {
          cmd_runonce(false);
          cmd_brake(true);
          state_runonce = false;
        }  
        return;
      }
      else
      {
        ROS_INFO("Undesirable goal state: %s",
            current_goal_state.toString().c_str());
        ROS_INFO("Client will abort the current path request, and await further "
            "requests or manual intervention.");
        fields.follow_waypoints_client->cancelGoal();
        goal_path.clear();
        reset_waypoints_path();
        request_error = true;
        error_mode_handle(amr_v3_msgs::Error::NAVIGATION_ERROR, NAV_ERROR, false);
        if (state_runonce) {
          cmd_runonce(false);
          cmd_brake(true);
          state_runonce = false;
        }  
        return;
      }
    }
    // otherwise, mode is correct, nothing in queue, nothing else to do then
  }

  WriteLock dock_goal_lock(dock_goal_mutex);
  if (dock_goal.start_docking)
  {
    if (!docking)
      docking = true;

    // ROS_WARN("Sending autodock goal.");waypoints_path
    // docking = false;
    // return;

    if (!dock_goal.sent)
    {
      if (!state_runonce) {
        cmd_runonce(true);
        state_runonce = true;
      }      
      ROS_INFO("sending autodock goal.");
      fields.autodock_client->sendGoal(dock_goal.autodock_goal);
      // goal_path.front().sent = true;
      dock_goal.sent = true;
      return;
    }

    // Goals have been sent, check the goal states now
    GoalState current_goal_state = fields.autodock_client->getState();
    if (current_goal_state == GoalState::SUCCEEDED)
    {
      ROS_INFO("Autodock goal state: SUCCEEEDED.");
      reset_autodock_goal();
      if (state_runonce) {
          cmd_runonce(false);
          state_runonce = false;
      }  
      return;
    }
    else if (current_goal_state == GoalState::ACTIVE)
    {
      return;
    }
    else if (current_goal_state == GoalState::ABORTED)
    {
      dock_goal.aborted_count++;

      // TODO: parameterize the maximum number of retries.
      if (dock_goal.aborted_count < 3)
      {
        ROS_INFO("robot's autodock has aborted the current goal %d "
            "times, client will trying again...",
            dock_goal.aborted_count);
        fields.autodock_client->cancelGoal();
        dock_goal.sent = false;
        return;
      }
      else
      {
        ROS_INFO("robot's autodock has aborted the current goal %d "
            "times, please check that there is nothing in the way of the "
            "robot, client will abort the current docking request, and await "
            "further requests.",
            dock_goal.aborted_count);
        fields.autodock_client->cancelGoal();
        request_error = true;
        if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_CHARGE) {
          error_mode_handle(amr_v3_msgs::Error::DOCK_CHARGE_ERROR, DOCK_CHARGE_ERROR, false);
        } else if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_PICKUP)
        {
          error_mode_handle(amr_v3_msgs::Error::DOCK_PICKUP_ERROR, DOCK_PICKUP_ERROR, false);
        } else if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_DROPOFF)
        {
          error_mode_handle(amr_v3_msgs::Error::DOCK_DROPOFF_ERROR, DOCK_DROPOFF_ERROR, false);
        }
        reset_autodock_goal();
        if (state_runonce) {
          cmd_runonce(false);
          cmd_brake(true);
          state_runonce = false;
        }  
        return;
      }
    }
    else if (current_goal_state == GoalState::PREEMPTED) {
      ROS_INFO("Client was cancel the current docking, and await further "
          "requests or manual intervention.");
      reset_autodock_goal();
      if (state_runonce) {
        cmd_runonce(false);
        cmd_brake(true);
        state_runonce = false;
      }  
      return;
    }
    else
    {
      ROS_INFO("Undesirable goal state: %s",
          current_goal_state.toString().c_str());
      ROS_INFO("Client will abort the current docking request, and await further "
          "requests or manual intervention.");
      fields.autodock_client->cancelGoal();
      dock_goal.start_docking = false;
      docking = false;
      request_error = true;
      if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_CHARGE) {
        error_mode_handle(amr_v3_msgs::Error::DOCK_CHARGE_ERROR, DOCK_CHARGE_ERROR, false);
      } else if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_PICKUP)
      {
        error_mode_handle(amr_v3_msgs::Error::DOCK_PICKUP_ERROR, DOCK_PICKUP_ERROR, false);
      } else if (dock_goal.autodock_goal.mode == dock_goal.autodock_goal.MODE_DROPOFF)
      {
        error_mode_handle(amr_v3_msgs::Error::DOCK_DROPOFF_ERROR, DOCK_DROPOFF_ERROR, false);
      }
      reset_autodock_goal();
      if (state_runonce) {
          cmd_runonce(false);
          cmd_brake(true);
          state_runonce = false;
      }  
      return;
    }
  }
}

void ClientNode::doneCb(const actionlib::SimpleClientGoalState& state,
                        const follow_waypoints::FollowWaypointsResultConstPtr& result)
{
  return;
}

void ClientNode::activeCb()
{
  return;
}

void ClientNode::feedbackCb(const follow_waypoints::FollowWaypointsFeedbackConstPtr& feedback)
{
  WriteLock goal_path_lock(goal_path_mutex);
  if (waypoints_path.sent) {
    size_t n = goal_path.size() - feedback->remain_target_poses.poses.size();
    if (n <= goal_path.size() && n >= 1) {
      goal_path.erase(goal_path.begin(), goal_path.begin() + n);
      waypoints_path.follow_waypoints_path.target_poses = feedback->remain_target_poses;
      waypoints_path.follow_waypoints_path.obey_approach_speed_limit = feedback->remain_obey_approach_speed_limit;
      waypoints_path.follow_waypoints_path.approach_speed_limit = feedback->remain_approach_speed_limit;

      ROS_INFO("Feedback: robot is reached tolerance for current goal and will go to the next goal," 
      "update remaining target poses: %ld target poses", goal_path.size());
    }
  }
  return;
}

void ClientNode::cmd_runonce(bool run)
{
  std_msgs::Bool msg;
  msg.data = run;
  cmd_runonce_pub.publish(msg);
  return;
}

void ClientNode::cmd_brake(bool brake)
{
  std_msgs::Bool msg;
  msg.data = brake;
  cmd_brake_pub.publish(msg);
  return;
}

// void ClientNode::cmd_cancel(bool cancel)
// {
//   std_msgs::Bool msg;
//   msg.data = cancel;
//   cmd_cancel_pub.publish(msg);
//   return;
// }

void ClientNode::error_mode_handle(int32_t error_mode, const std::string& description, bool nolog)
{
  auto error = amr_v3_msgs::ErrorStamped();
  error.header.stamp = ros::Time::now();
  error.error.code = error_mode;
  error.error.description = description;
  error.error.nolog = nolog;
  mode_error_pub.publish(error);
} 

void ClientNode::reset_waypoints_path()
{
  waypoints_path.follow_waypoints_path.target_poses.poses.clear();
  waypoints_path.sent = false;
  waypoints_path.aborted_count = 0;
  return;
}

void ClientNode::reset_autodock_goal()
{
  dock_goal.start_docking = false;
  dock_goal.sent = false;
  dock_goal.aborted_count = 0;
  docking = false;
  return;
}


void ClientNode::update_thread_fn()
{
  while (node->ok())
  {
    update_rate->sleep();
    ros::spinOnce();

    get_robot_transform();

    read_requests();

    handle_requests();
  }
}

void ClientNode::publish_thread_fn()
{
  while (node->ok())
  {
    publish_rate->sleep();

    publish_robot_state();
  }
}

} // namespace ros1
} // namespace free_fleet
