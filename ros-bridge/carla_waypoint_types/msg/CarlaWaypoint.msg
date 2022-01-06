#
# Copyright (c) 2020 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

int32 road_id
int32 section_id
int32 lane_id
bool is_junction
float32 s
float32 lane_width
string lane_change
carla_waypoint_types/CarlaLaneMarking right_lane_marking
carla_waypoint_types/CarlaLaneMarking left_lane_marking
geometry_msgs/Pose pose