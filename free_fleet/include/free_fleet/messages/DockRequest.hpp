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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__DOCKREQUEST_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__DOCKREQUEST_HPP

#include "Location.hpp"
#include "DockMode.hpp"

namespace free_fleet {
namespace messages {

struct DockRequest
{
  std::string fleet_name;
  std::string robot_name;
  Location destination;
  std::string task_id;
  DockMode dock_mode;
  float distance_go_out;
  bool custom_docking;
  int16_t rotate_to_dock;
  int16_t rotate_angle;
  int16_t rotate_orientation;
};

} // namespace messages
} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__DOCKREQUEST_HPP
