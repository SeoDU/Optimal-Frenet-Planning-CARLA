#!/usr/bin/env python
"""
Evaluation of local planning performance
"""
import rospy
import time
import math
import carla
import atexit
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from carla_waypoint_types.srv import GetWaypoint
import carla_ros_bridge.transforms as trans


class Evaluator():
    def __init__(self, role_name):
        """
        Constructor
        """
        self._global_plan = None
        self._current_pose = None
        self._start_time = None
        self._start_time_ROS = None
        self._end_time = None
        self._end_time_ROS = None
        self._current_waypoint = None
        self._route_length = None
        self._current_speed = None
        self._on_goal_start_timer = None
        self._collision_sensor = None
        self._lane_invasion_sensor = None
        self._odom_initialized = False
        self._route_assigned = False

        # get world and map for finding actors and waypoints
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self._world = client.get_world()
        self._autopilot = {}

        # results
        self._mission_complete = False
        self._collision_counter = 0
        self._invasion_counter = 0
        self._distance_travelled = 0.0
        self._progress = 0.0

        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_handler)

        self._route_subscriber = rospy.Subscriber(
            "/carla/{}/waypoints".format(role_name), Path, self.path_handler)

        self._get_waypoint_client = rospy.ServiceProxy(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(role_name), GetWaypoint)

        self.goal_subscriber = rospy.Subscriber(
            "/carla/{}/goal".format(role_name), PoseStamped, self.on_goal)

        self._goal_publisher = rospy.Publisher(
            "/cara/{}/goal_marker".format(role_name), Marker, queue_size=1, latch=True)

    def __del__(self):
        self.output_result()

    def on_goal(self, goal):
        self._goal_point = goal.pose.position
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = 2
        marker.id = 0
        marker.action = 0
        marker.pose.position = goal.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.color.r = 1.0
        self._goal_publisher.publish(marker)

    def odometry_handler(self, odom):
        """
        callback on odometry
        """
        if not self._odom_initialized:
            self._start_time = time.time()
            self._start_time_ROS = rospy.get_rostime().to_sec()

        if self._current_pose:
            delta = self.point_distance(odom.pose.pose.position, self._current_pose.position)
            self._distance_travelled += delta

        self._current_pose = odom.pose.pose
        self._current_speed = math.sqrt(odom.twist.twist.linear.x ** 2 +
                                        odom.twist.twist.linear.y ** 2 +
                                        odom.twist.twist.linear.z ** 2) * 3.6
        self._odom_initialized = True

    def path_handler(self, path):
        """
        callback on global path
        """
        self._global_plan = path
        self._route_assigned = True
        self._route_length = 0
        min_dist = float('inf')
        min_idx = -1
        if len(self._global_plan.poses) > 1:
            for i in range(len(self._global_plan.poses)):
                curr_waypoint = self._global_plan.poses[i].pose.position
                dist = self.point_distance(self._goal_point, curr_waypoint)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i

            self._route_length = min_idx + 1

            rospy.loginfo("New plan with {} waypoints (real: {} ) received.".format(len(self._global_plan.poses),
                                                                                    self._route_length))

    def get_waypoint(self, location):
        """
        Helper to get waypoint from a ros service
        """
        try:
            response = self._get_waypoint_client(location)
            return response.waypoint
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            if not rospy.is_shutdown:
                rospy.logwarn("Service call failed: {}".format(e))

    def collision_handler(self, event):
        """
        Triggered when collision occurs
        """
        print("collision")
        self._collision_counter += 1

    def lane_invasion_handler(self, event):
        """
        Triggered when lane invasion occurs
        """
        lane_types = set(x.type for x in event.crossed_lane_markings)
        lanes = [str(x).split()[-1] for x in lane_types]
        for l in lanes:
            print('Crossed line %s' % l)
            if 'Solid' in l:
                self._invasion_counter += 1
                break

    def output_result(self):
        if not self._end_time or not self._start_time:
            return

        print("\x1b[6;30;42m------Simulation Result------")
        print("Time: {} s".format(self._end_time_ROS - self._start_time_ROS))
        print()
        print("Distance travelled: {} m".format(self._distance_travelled))
        print("Mission progress: {}% ".format(self._progress))
        print("Mission completed: {}".format(self._mission_complete))
        collision_per_m = 0.0 if self._distance_travelled == 0.0 else self._collision_counter / self._distance_travelled
        invasion_per_m = 0.0 if self._distance_travelled == 0.0 else self._invasion_counter / self._distance_travelled
        print("# Collision per m: {}".format(collision_per_m))
        print("# Invasion per m: {}".format(invasion_per_m))
        print("-----------------------------\x1b[0m")
        rospy.signal_shutdown("")

    def run(self):
        """
        run evaluator
        """
        while not rospy.is_shutdown():
            if self._odom_initialized:

                print("------------------------------------")

                # initialize collision sensor
                if not self._collision_sensor:
                    client = carla.Client('localhost', 2000)
                    client.set_timeout(10.0)
                    world = client.get_world()
                    blueprint_library = world.get_blueprint_library()
                    actor_list = world.get_actors()
                    ego_vehicle = None
                    for actor in actor_list:
                        if "role_name" in actor.attributes:
                            if actor.attributes["role_name"] == 'ego_vehicle':
                                ego_vehicle = actor
                    if ego_vehicle:
                        self._collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                                                   carla.Transform(), attach_to=ego_vehicle)
                        self._lane_invasion_sensor = world.spawn_actor(
                            blueprint_library.find('sensor.other.lane_invasion'), carla.Transform(),
                            attach_to=ego_vehicle)
                        self._collision_sensor.listen(lambda event: self.collision_handler(event))
                        self._lane_invasion_sensor.listen(lambda event: self.lane_invasion_handler(event))
                    else:
                        print("Waiting for ego vehicle for attaching collision sensor")
                        continue

                        # Time evaluation
                self._end_time = time.time()
                self._end_time_ROS = rospy.get_rostime().to_sec()
                time_elapsed = self._end_time - self._start_time
                ROS_time_elapsed = self._end_time_ROS - self._start_time_ROS
                # if time_elapsed >= 60 * 5.0:
                #     print("Time is up")
                #     self.output_result()
                # else:
                #     print("Timer: {} sec".format(time_elapsed))
                if ROS_time_elapsed >= 60 * 5.0:
                    print("Time is up")
                    self.output_result()
                else:
                    # print("Timer: {} sec".format(time_elapsed))
                    print("ROS Timer: {} sec".format(ROS_time_elapsed))
                # Progress evaluation
                if self._route_assigned:
                    self._current_waypoint = self.get_waypoint(self._current_pose.position)
                    min_dist = float('inf')
                    min_idx = -1
                    for i in range(len(self._global_plan.poses)):
                        dist = self.point_distance(self._current_waypoint.pose.position,
                                                   self._global_plan.poses[i].pose.position)
                        if dist < min_dist:
                            min_dist = dist
                            min_idx = i
                    if min_idx >= 0 and self._route_length > 0:
                        self._progress = float(min_idx + 1) / self._route_length * 100.0
                        if self._progress > 100.0:
                            self._progress = 100.0
                        print("Progress: {} %".format(self._progress))

                # on goal evaluation
                distance_to_goal = self.point_distance(self._current_pose.position, self._goal_point)
                print("Distance to goal {} m".format(distance_to_goal))
                print("current speed  {} km/h".format(self._current_speed))

                if distance_to_goal <= 4.0 and self._current_speed < 0.001:
                    if not self._on_goal_start_timer:
                        self._on_goal_start_timer = rospy.get_rostime().to_sec()
                    else:
                        time_diff = rospy.get_rostime().to_sec() - self._on_goal_start_timer
                        print("On goal timer: {} sec".format(time_diff))
                        if time_diff >= 2.0:
                            print("Mission complete")
                            self._mission_complete = True
                            self.output_result()
                else:
                    self._on_goal_start_timer = None

                # invasion and collision
                print("Invasion: {}".format(self._invasion_counter))
                print("Collsion: {}".format(self._collision_counter))
                print("------------------------------------")

                # set autopilot to move
                actor_list = self._world.get_actors()
                for actor in actor_list:
                    if "role_name" in actor.attributes:
                        if actor.attributes["role_name"] == 'autopilot':
                            if not actor.id in self._autopilot:
                                self._autopilot[actor.id] = False
                            if not self._autopilot[actor.id]:
                                carla_transform = actor.get_transform()
                                ros_transform = trans.carla_transform_to_ros_pose(carla_transform)
                                distance = self.point_distance(ros_transform.position, self._current_pose.position)
                                # print("Distance to pilot {}: {}".format(actor.id, distance))
                                if distance < 20.0:
                                    self._autopilot[actor.id] = True
                                    actor.set_autopilot(True)

    def point_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def exit_handler(evaluator):
    evaluator.output_result()


def main():
    rospy.init_node('evaluator', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    evaluator = Evaluator(role_name)
    atexit.register(exit_handler, evaluator)
    try:
        evaluator.run()
    finally:
        del evaluator
        rospy.loginfo("Done")


if __name__ == '__main__':
    main()