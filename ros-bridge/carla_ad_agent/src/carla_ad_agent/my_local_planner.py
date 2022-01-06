#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

from sensor_msgs.msg import PointCloud2
from collections import deque
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from carla_waypoint_types.srv import GetWaypoint
from carla_msgs.msg import CarlaEgoVehicleControl
from vehicle_pid_controller import VehiclePIDController  # pylint: disable=relative-import
from misc import distance_vehicle  # pylint: disable=relative-import
import carla
import carla_ros_bridge.transforms as trans
from visualization_msgs.msg import MarkerArray, Marker
from carla_waypoint_types.srv import GetWaypointResponse, GetWaypoint
import frenet_trajectory
# from vehicle_mpc_controller import MPCcontroller
import copy

CAR_EXTENT = 1.8

class Obstacle:
    def __init__(self):
        self.id = -1  # actor id
        self.vx = 0.0  # velocity in x direction
        self.vy = 0.0  # velocity in y direction
        self.vz = 0.0  # velocity in z direction
        self.ros_transform = None  # transform of the obstacle in ROS coordinate
        self.carla_transform = None  # transform of the obstacle in Carla world coordinate
        self.bbox = None  # Bounding box w.r.t ego vehicle's local frame


class MyLocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self, role_name, opt_dict=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param role_name: name of the actor
        :param opt_dict: dictionary of arguments with the following semantics:

            target_speed -- desired cruise speed in Km/h

            sampling_radius -- search radius for next waypoints in seconds: e.g. 0.5 seconds ahead

            lateral_control_dict -- dictionary of arguments to setup the lateral PID controller
                                    {'K_P':, 'K_D':, 'K_I'}

            longitudinal_control_dict -- dictionary of arguments to setup the longitudinal
                                         PID controller
                                         {'K_P':, 'K_D':, 'K_I'}
        """
        self.target_route_point = None
        self._current_waypoint = None
        self._vehicle_controller = None
        self._waypoints_queue = deque(maxlen=20000)
        self._buffer_size = 40
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self._vehicle_yaw = None
        self._current_speed = None
        self._current_pose = None
        self._obstacles = []

        # get world and map for finding actors and waypoints
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.map = self.world.get_map()

        self.brake_point_publisher = rospy.Publisher(
            "/next_target", PointStamped, queue_size=1)

        self.last_buffer_publisher = rospy.Publisher(
            "/last_waypoint", PointStamped, queue_size=1)

        self.left_center_publisher = rospy.Publisher(
            "/left_center", PointStamped, queue_size=1)
        self.right_center_publisher = rospy.Publisher(
            "/right_center", PointStamped, queue_size=1)

        ########## ADDED ##########
        self._lane_left_publisher = rospy.Publisher(
            "/lane_left", Marker, queue_size=1)
        self._lane_right_publisher = rospy.Publisher(
            "/lane_right", Marker, queue_size=1)

        self.frenet_traj_publisher = rospy.Publisher(
            "/frenet_traj", MarkerArray, queue_size=100)
        # self.frenet_traj_right_publisher = rospy.Publisher(
        #     "/frenet_traj_right", Marker, queue_size=1)

        self.left_node_arr = MarkerArray()
        self.right_node_arr = MarkerArray()

        # self.tmp_publisher = rospy.Publisher(
        #     "/tmp", Marker, queue_size=10)
        self.lane_id = 0

        self.total_waypoints = []
        self.wp_s = 0
        self.first_run = True

        self.left_lane_node = Marker()
        self.left_lane_node.header.frame_id = "map"
        self.left_lane_node.id = self.lane_id
        self.left_lane_node.header.stamp = rospy.Time.now()
        self.left_lane_node.ns = "lane"

        self.left_lane_node.pose.orientation.x = 0.0
        self.left_lane_node.pose.orientation.y = 0.0
        self.left_lane_node.pose.orientation.z = 0.0
        self.left_lane_node.pose.orientation.w = 1.0
        self.left_lane_node.pose.position.x = 0.0
        self.left_lane_node.pose.position.y = 0.0
        self.left_lane_node.pose.position.z = 0.0
        self.left_lane_node.scale.x = 0.1

        self.left_lane_node.type = Marker.LINE_STRIP  # ARROW : 0, SPHERE : 2
        self.left_lane_node.action = Marker.ADD  # ADD

        self.right_lane_node = Marker()
        self.right_lane_node.header.frame_id = "map"
        self.right_lane_node.id = self.lane_id + 1
        self.right_lane_node.header.stamp = rospy.Time.now()
        self.right_lane_node.ns = "lane"

        self.right_lane_node.type = Marker.LINE_STRIP  # ARROW : 0, SPHERE : 2
        self.right_lane_node.action = Marker.ADD  # ADD
        self.right_lane_node.pose.orientation.x = 0.0
        self.right_lane_node.pose.orientation.y = 0.0
        self.right_lane_node.pose.orientation.z = 0.0
        self.right_lane_node.pose.orientation.w = 1.0
        self.right_lane_node.pose.position.x = 0.0
        self.right_lane_node.pose.position.y = 0.0
        self.right_lane_node.pose.position.z = 0.0
        self.right_lane_node.scale.x = 0.1

        self.traj_nodes = MarkerArray()
        self.path_buf = []

        self.cc_sub = rospy.Subscriber("/ccs",Float32MultiArray,self.cc_callback )
        self.cc = []

        # self.cluster0_subscriber = rospy.Subscriber("/cluster_0",PointCloud2,self.lidar_callback )
        # self.cluster1_subscriber = rospy.Subscriber("/cluster_1", PointCloud2, self.lidar_callback)
        # self.cluster2_subscriber = rospy.Subscriber("/cluster_2", PointCloud2, self.lidar_callback)
        # self.cluster3_subscriber = rospy.Subscriber("/cluster_3", PointCloud2, self.lidar_callback)
        # self.cluster4_subscriber = rospy.Subscriber("/cluster_4", PointCloud2, self.lidar_callback)
        # self.cluster5_subscriber = rospy.Subscriber("/cluster_5", PointCloud2, self.lidar_callback)
        ###########################

        rospy.wait_for_service('/carla_waypoint_publisher/{}/get_waypoint'.format(role_name))
        self._get_waypoint_client = rospy.ServiceProxy(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(role_name), GetWaypoint)

        self.goal_reached = False

        # initializing controller
        self._init_controller(opt_dict)
    def cc_callback(self,msg):
        self.cc = []
        for i in range(6):
            ob = Obstacle()
            ob.id = -1
            ob.carla_transform = None
            ob.ros_transform = Pose()
            ob.ros_transform.position.x = msg.data[3 * i]
            ob.ros_transform.position.y = msg.data[3 * i + 1]
            ob.ros_transform.position.z = msg.data[3 * i + 2]

            ob.vx = 0
            ob.vy = 0
            ob.vz = 0
            ob.bbox = carla.BoundingBox()
            ob.bbox.extent.x = 1.0
            ob.bbox.extent.y = 1.0
            self._obstacles.append(ob)


    def get_traj_node(self):
        traj_node = Marker()
        traj_node.header.frame_id = "map"

        traj_node.type = Marker.LINE_STRIP  # ARROW : 0, SPHERE : 2
        traj_node.action = Marker.ADD  # ADD
        traj_node.id = self.lane_id + 2 + len(self.traj_nodes.markers)
        traj_node.header.stamp = rospy.Time.now()
        traj_node.ns = "traj"

        traj_node.pose.orientation.x = 0.0
        traj_node.pose.orientation.y = 0.0
        traj_node.pose.orientation.z = 0.0
        traj_node.pose.orientation.w = 1.0
        traj_node.pose.position.x = 0.0
        traj_node.pose.position.y = 0.0
        traj_node.pose.position.z = 0.0
        traj_node.scale.x = 0.1
        return traj_node

    def get_obstacles(self, location, range):
        """
        Get a list of obstacles that are located within a certain distance from the location.

        :param      location: queried location
        :param      range: search distance from the queried location
        :type       location: geometry_msgs/Point
        :type       range: float or double
        :return:    None
        :rtype:     None
        """
        self._obstacles = []
        actor_list = self.world.get_actors()
        for actor in actor_list:
            if "role_name" in actor.attributes:
                if actor.attributes["role_name"] == 'autopilot' or actor.attributes["role_name"] == "static":
                    carla_transform = actor.get_transform()
                    ros_transform = trans.carla_transform_to_ros_pose(carla_transform)
                    x = ros_transform.position.x
                    y = ros_transform.position.y
                    z = ros_transform.position.z
                    distance = math.sqrt((x - location.x) ** 2 + (y - location.y) ** 2)
                    if distance < range:
                        # print("obs distance: {}").format(distance)
                        ob = Obstacle()
                        ob.id = actor.id
                        ob.carla_transform = carla_transform
                        ob.ros_transform = ros_transform
                        ob.vx = actor.get_velocity().x
                        ob.vy = actor.get_velocity().y
                        ob.vz = actor.get_velocity().z
                        ob.bbox = actor.bounding_box  # in local frame
                        # print("x: {}, y: {}, z:{}").format(x, y, z)
                        # print("bbox x:{} y:{} z:{} ext: {} {} {}".format(ob.bbox.location.x, ob.bbox.location.y, ob.bbox.location.z, ob.bbox.extent.x, ob.bbox.extent.y, ob.bbox.extent.z))
                        self._obstacles.append(ob)

    def check_obstacle(self, point, obstacle):
        """
        Check whether a point is inside the bounding box of the obstacle

        :param      point: a location to check the collision (in ROS frame)
        :param      obstacle: an obstacle for collision check
        :type       point: geometry_msgs/Point
        :type       obstacle: object Obstacle
        :return:    true or false
        :rtype:     boolean
        """
        carla_location = carla.Location()
        carla_location.x = point.x
        carla_location.y = -point.y
        carla_location.z = point.z

        vertices = obstacle.bbox.get_world_vertices(obstacle.carla_transform)

        vx = [v.x for v in vertices]
        vy = [v.y for v in vertices]
        vz = [v.z for v in vertices]
        return carla_location.x >= min(vx) and carla_location.x <= max(vx) \
               and carla_location.y >= min(vy) and carla_location.y <= max(vy) \
               and carla_location.z >= min(vz) and carla_location.z <= max(vz)

    def get_coordinate_lanemarking(self, position):
        """
        Helper to get adjacent waypoint 2D coordinates of the left and right lane markings
        with respect to the closest waypoint

        :param      position: queried position
        :type       position: geometry_msgs/Point
        :return:    left and right waypoint in numpy array
        :rtype:     tuple of geometry_msgs/Point (left), geometry_msgs/Point (right)
        """
        # get waypoints along road
        current_waypoint = self.get_waypoint(position)
        if not current_waypoint:
            return None, None, None, None

        waypoint_xodr = self.map.get_waypoint_xodr(current_waypoint.road_id, current_waypoint.lane_id,
                                                   current_waypoint.s)

        if not waypoint_xodr:
            return None,None,None,None

        # find two orthonormal vectors to the direction of the lane
        yaw = math.pi - waypoint_xodr.transform.rotation.yaw * math.pi / 180.0
        norm_v = np.array([math.cos(yaw), math.sin(yaw)])
        # norm_v = np.array([1, math.tan(yaw)])
        # norm_v /= np.linalg.norm(norm_v)
        right_v = np.array([-norm_v[1], norm_v[0]])
        left_v = np.array([norm_v[1], -norm_v[0]])

        # find two points that are on the left and right lane markings
        half_width = current_waypoint.lane_width / 2.0
        left_waypoint = np.array(
            [current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + half_width * left_v
        right_waypoint = np.array(
            [current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + half_width * right_v

        left_road_center = np.array(
            [current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + 2 * half_width * left_v
        right_road_center = np.array(
            [current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + 2 * half_width * right_v

        ros_left_waypoint = Point()
        ros_right_waypoint = Point()
        ros_left_center = Point()
        ros_right_center = Point()

        ros_left_waypoint.x = left_waypoint[0]
        ros_left_waypoint.y = left_waypoint[1]
        ros_right_waypoint.x = right_waypoint[0]
        ros_right_waypoint.y = right_waypoint[1]

        ros_left_center.x = left_road_center[0]
        ros_left_center.y = left_road_center[1]
        ros_right_center.x = right_road_center[0]
        ros_right_center.y = right_road_center[1]

        return ros_left_waypoint, ros_right_waypoint, ros_left_center, ros_right_center

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

    def odometry_updated(self, odo):
        """
        Callback on new odometry
        """
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6

        self._current_pose = odo.pose.pose
        quaternion = (
            odo.pose.pose.orientation.x,
            odo.pose.pose.orientation.y,
            odo.pose.pose.orientation.z,
            odo.pose.pose.orientation.w
        )
        _, _, self._vehicle_yaw = euler_from_quaternion(quaternion)

    def _init_controller(self, opt_dict):
        """
        Controller initialization.

        :param opt_dict: dictionary of arguments.
        :return:
        """
        # default params
        args_lateral_dict = {
            'K_P': 1.95,  # 1.95
            'K_D': 0.01, # 0.01
            'K_I': 1.4} # 1.4
        args_longitudinal_dict = {
            'K_P': 0.2,  # 0.2
            'K_D': 0.00, # 0.05
            'K_I': 0.00} # 0.1

        # parameters overload
        if opt_dict:
            if 'lateral_control_dict' in opt_dict:
                args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                args_longitudinal_dict = opt_dict['longitudinal_control_dict']

        self._vehicle_controller = VehiclePIDController(args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict)
        # self._vehicle_controller = MPCcontroller()

    def set_global_plan(self, current_plan):
        """
        set a global plan to follow
        """
        self.target_route_point = None
        self._waypoint_buffer.clear()
        self._waypoints_queue.clear()
        for elem in current_plan:
            new_wp = [elem.pose.position.x, elem.pose.position.y]
            if new_wp not in self.total_waypoints:
                self.total_waypoints.append(new_wp)
            self._waypoints_queue.append(elem.pose)

        self.wp_s = np.zeros(len(self.total_waypoints))
        for i in range(len(self.total_waypoints) - 1):
            self.wp_s[i], _ = frenet_trajectory.get_frenet_coord(self.total_waypoints[i][0], self.total_waypoints[i][1],
                                                                 self.total_waypoints)

        self.wp_s[-1] = self.wp_s[-2] \
                        + frenet_trajectory.get_dist(self.total_waypoints[-2][0], self.total_waypoints[-2][1],
                                                     self.total_waypoints[-1][0], self.total_waypoints[-1][1])


    def run_step(self, target_speed, current_speed, current_pose):
        """
        Execute one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        target_speed = 90
        if not self._waypoint_buffer and not self._waypoints_queue:
            control = CarlaEgoVehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False

            rospy.loginfo("Route finished.")
            return control, True

        #   Buffering the waypoints
        if len(self._waypoint_buffer) < self._buffer_size:
            for i in range(self._buffer_size - len(self._waypoint_buffer)):
                if self._waypoints_queue:
                    wp_popped = self._waypoints_queue.popleft()
                    # if not self.first_run:
                    #     print "** before new waypoint added", self.initial_conditions['wp']
                    #     self.initial_conditions['wp'] = \
                    #         np.append(self.initial_conditions['wp'],
                    #                   [[wp_popped.position.x,wp_popped.position.y]],axis=0)
                    #     print "** new waypoint added", self.initial_conditions['wp']
                    self._waypoint_buffer.append(wp_popped)
                else:
                    break



        # current vehicle waypoint
        self._current_waypoint = self.get_waypoint(current_pose.position)

        # get a list of obstacles surrounding the ego vehicle
        self.get_obstacles(current_pose.position, 70.0)

        # get current s,d coordinates on the frenet frame
        cur_s, cur_d = frenet_trajectory.get_frenet_coord(current_pose.position.x, current_pose.position.y,
                                                          self.total_waypoints)

        last_waypoint = PointStamped()
        last_waypoint.header.frame_id = "map"
        last_waypoint.point.x = self._waypoint_buffer[-1].position.x
        last_waypoint.point.y = self._waypoint_buffer[-1].position.y
        last_waypoint.point.z = self._waypoint_buffer[-1].position.z
        self.last_buffer_publisher.publish(last_waypoint)

        # get type of lanes by current position and last waypoint
        rsp_cur = self.get_lane_waypoint(
            carla.Location(x=current_pose.position.x, y=current_pose.position.y,
                           z=current_pose.position.z)
        )
        rsp_wp = self.get_lane_waypoint(
            carla.Location(x=self._waypoint_buffer[-1].position.x, y=self._waypoint_buffer[-1].position.y,
                           z=self._waypoint_buffer[-1].position.z)
        )

        # get left and right position for lanes and center
        # If left or right lane is solid, erase them.
        left_wp, right_wp, left_center_wp, right_center_wp = self.get_coordinate_lanemarking(self._waypoint_buffer[-1].position)
        if "Solid" in rsp_wp.waypoint.left_lane_marking.type : # or "NONE" in rsp_wp.waypoint.left_lane_marking.type
            left_center_wp = None
        if "Solid" in rsp_wp.waypoint.right_lane_marking.type : # or "NONE" in rsp_wp.waypoint.left_lane_marking.type
            right_center_wp = None

        # By default, destination candidate should have the global path waypoint.
        DF_SET = [0]

        # if car is not in right next lane, df should be ommited
        if right_center_wp is not None and cur_d < self._current_waypoint.lane_width / 2.0:
            DF_SET.append(-self._current_waypoint.lane_width)
        if left_center_wp is not None and cur_d > -self._current_waypoint.lane_width / 2.0:
            DF_SET.append(self._current_waypoint.lane_width)

        # if the car should cross the solid line to go candidate in current pose, remove.
        for df in DF_SET:
            if df - cur_d > self._current_waypoint.lane_width / 2.0 and "Solid" in rsp_cur.waypoint.left_lane_marking.type:
                DF_SET.remove(df)
                break
            if df - cur_d < -self._current_waypoint.lane_width / 2.0 and "Solid" in rsp_cur.waypoint.right_lane_marking.type:
                DF_SET.remove(df)
                break

        # if "Solid" not in rsp_wp.waypoint.left_lane_marking.type or "Solid" not in rsp_wp.waypoint.right_lane_marking.type:
        #     print  "WP",rsp_wp.waypoint.left_lane_marking.type, "/" , rsp_wp.waypoint.right_lane_marking.type
        # if "Solid" not in rsp_cur.waypoint.left_lane_marking.type or "Solid" not in rsp_cur.waypoint.right_lane_marking.type:
        #     print  "CUR",rsp_cur.waypoint.left_lane_marking.type, "/", rsp_cur.waypoint.right_lane_marking.type
        # print "DF_SET:", DF_SET

        # Draw lanes
        left_cur, right_cur, _, _ = self.get_coordinate_lanemarking(
            current_pose.position)

        if left_cur is not None:
            self.left_lane_node.points.append(left_cur)
            self.set_lane_marker(self.left_lane_node, left_cur, rsp_cur.waypoint.left_lane_marking.type)
            self._lane_left_publisher.publish(self.left_lane_node)

        if right_cur is not None:
            self.right_lane_node.points.append(right_cur)
            self.set_lane_marker(self.right_lane_node, right_cur, rsp_cur.waypoint.right_lane_marking.type)
            self._lane_right_publisher.publish(self.right_lane_node)

        self.si = cur_s
        self.sf_d = target_speed
        self.di = cur_d
        if self.first_run:
            _, _, yaw_road = frenet_trajectory.get_cartesian(cur_s, cur_d, self.total_waypoints, self.wp_s)
            # yaw = np.arctan2((current_pose.position.y - self.total_waypoints[0][1]),
            #                  (current_pose.position.x - self.total_waypoints[0][0]))
            # yawi = 3.0 * np.pi / 2.0 + yaw_road
            self.si_d = current_speed * np.cos(self._vehicle_yaw - yaw_road) #- current_speed * np.cos(yawi)
            self.di_d = current_speed * np.sin(self._vehicle_yaw - yaw_road) #- current_speed * np.sin(yawi)
            self.si_dd = 0
            self.sf_dd = 0
            self.di_dd = 0
            self.df_d = 0
            self.df_dd = 0
            self.opt_d = self.di
            self.first_run = False

        obs = []
        # print "---------obstacles---------"
        for ob in self._obstacles:
            obs.append([ob.ros_transform.position,
                        ob.bbox.extent.x, ob.bbox.extent.y, ob.vx, ob.vy])
            # print "vx,vy", ob.vx, ob.vy
            #print ob.ros_transform.position, ob.bbox.extent
        # print "---------------------------"

        path,opt_ind = frenet_trajectory.frenet_optimal_planning(target_speed*1000/3600, DF_SET, self.si, self.si_d, self.si_dd,
                                                self.sf_d, self.sf_dd, self.di, self.di_d, self.di_dd, self.df_d, self.df_dd, obs, self.total_waypoints, self.wp_s, self.opt_d)

        if path:
            #print "opt cost", path[opt_ind].c_tot
            # Future work: Change si_d, di_d to equation : -current_speed * np.cos(yaw)
            # yaw = 3.0*np.pi/2.0 + path.yaw or path.yaw - yaw_road ?
            self.si = cur_s #path[opt_ind].s[0]
            #s_speed = current_speed * np.cos(self._vehicle_yaw - path[opt_ind].yaw[0])
            self.si_d = current_speed * np.cos(self._vehicle_yaw - path[opt_ind].yaw[0])#path[opt_ind].s_d[1]
            self.si_dd = path[opt_ind].s_dd[1]
            self.di = cur_d #path[opt_ind].d[0]
            self.di_d = path[opt_ind].d_d[1]
            self.di_dd = path[opt_ind].d_dd[1]
            self.opt_d = path[opt_ind].d[-1]
            #self._vehicle_yaw = path[opt_ind].yaw[0]
            # print "left_center", left_center
            # print "right_center", right_center
            #print "pathxy", path[0].x[-1], path[0].y[-1]

            self.traj_nodes = MarkerArray()
            # print "path len/ path_opt len: ", len(path), "/", len(path[opt_ind].x)
            for i in range(len(path)):
                new_traj_node = self.get_traj_node()
                for j in range(len(path[i].x)):
                    traj_p = Point()
                    traj_p.x = path[i].x[j]
                    traj_p.y = path[i].y[j]
                    new_traj_node.points.append(traj_p)
                if "Solid" not in rsp_cur.waypoint.left_lane_marking.type and len(DF_SET) == 2: # left, center traj
                    self.set_lane_marker(new_traj_node, traj_p, rsp_cur.waypoint.left_lane_marking.type)
                elif "Solid" not in rsp_cur.waypoint.right_lane_marking.type and len(DF_SET) == 2: # right, center traj
                    self.set_lane_marker(new_traj_node, traj_p, rsp_cur.waypoint.right_lane_marking.type)
                elif len(DF_SET) == 1: # center only traj
                    self.set_lane_marker(new_traj_node, traj_p, "WHITE")
                else: # right, left, center traj
                    self.set_lane_marker(new_traj_node, traj_p, "WHITE")

                if i == opt_ind:
                    new_traj_node.scale.x = 0.5
                    new_traj_node.color.r = 0.4
                    new_traj_node.color.g = 0.0
                    new_traj_node.color.b = 0.6
                    new_traj_node.color.a = 1.0
                self.traj_nodes.markers.append(new_traj_node)
            self.frenet_traj_publisher.publish(self.traj_nodes)

        # rsp_dst = self.get_lane_waypoint(
        #
        # )

        # rsp_cur = self.get_lane_waypoint(
        #     carla.Location(x=self.total_waypoints[clp][0], y=self.total_waypoints[clp][1],
        #                    z=current_pose.position.z)
        # )
        # dx = self._waypoint_buffer[-1].position.x - self.total_waypoints[clp][0]
        # dy = self._waypoint_buffer[-1].position.y - self.total_waypoints[clp][1]
        #print "cur_d", cur_d, self._current_waypoint.lane_width / 2.0


        # if right_center_wp is not None and cur_d <= -self._current_waypoint.lane_width / 2.0:
        #     rsp_arg = carla.Location(x=right_center.x, y=right_center.y,
        #                    z=right_center.z)
        #     left, right, left_center, right_center = self.get_coordinate_lanemarking(right_center)
        #
        # elif left_center is not None and  cur_d > self._current_waypoint.lane_width / 2.0:
        #     rsp_arg = carla.Location(x=left_center.x, y=left_center.y,
        #                              z=left_center.z)
        #     left, right, left_center, right_center = self.get_coordinate_lanemarking(left_center)
        #     dfset_bias = self._current_waypoint.lane_width
        # else:
        #     rsp_arg = carla.Location(x=self._waypoint_buffer[-1].position.x, y=self._waypoint_buffer[-1].position.y,
        #                    z=self._waypoint_buffer[-1].position.z)
        #     dfset_bias = 0


        # rsp = self.get_lane_waypoint(
        #     rsp_arg
        #     # carla.Location(x=self.target_route_point.position.x, y=self.target_route_point.position.y,
        #     #                z=self.target_route_point.position.z)
        #     # carla.Location(x=self._waypoint_buffer[-1].position.x, y=self._waypoint_buffer[-1].position.y,
        #     #                z=self._waypoint_buffer[-1].position.z)
        #     # carla.Location(x=current_pose.position.x + dx, y=current_pose.position.y + dy,
        #     #                z=current_pose.position.z)
        # )

        # # Example 1: get two waypoints on the left and right lane marking w.r.t current pose
        # tmp = Pose(position = Point(x=current_pose.position.x + dx, y=current_pose.position.y + dy)
        #            orientation=Quaternion(0,0,0,1))


        # left, right, left_center, right_center = self.get_coordinate_lanemarking(
        #     Point(x=current_pose.position.x + dx, y=current_pose.position.y + dy)
        # )


        # if left_center is not None:
        #     left_cpoint = PointStamped()
        #     left_cpoint.header.frame_id = "map"
        #     left_cpoint.point = left_center
        #     self.left_center_publisher.publish(left_cpoint)
        #     if "Solid" not in rsp.waypoint.left_lane_marking.type and "Solid" not in rsp_cur.waypoint.left_lane_marking.type:
        #         DF_SET.append(self._current_waypoint.lane_width)
        #
        # if right_center is not None:
        #     right_cpoint = PointStamped()
        #     right_cpoint.header.frame_id = "map"
        #     right_cpoint.point = right_center
        #     self.right_center_publisher.publish(right_cpoint)
        #     if "Solid" not in rsp.waypoint.right_lane_marking.type and "Solid" not in rsp_cur.waypoint.right_lane_marking.type:
        #         DF_SET.append(-self._current_waypoint.lane_width)
        #
        # for i in range(len(DF_SET)):
        #     DF_SET[i] += dfset_bias

        #print DF_SET

        # # Example 2: check obstacle collision
        #        print("\x1b[6;30;33m------Example 2------\x1b[0m")
        #        point = Point()
        #        point.x = 100.0
        #        point.y = 100.0
        #        point.z = 1.5
        #        for ob in self._obstacles:
        #            print("id: {}, collision: {}".format(ob.id, self.check_obstacle(point, ob)))

        # print "total wps len", len(self.total_waypoints)
        # print "Current position ", current_pose.position
        # print "Wp buffer: ", self._waypoint_buffer[0]

        # print "---------------------------------"
        # for i in range(10):
        #     print self.total_waypoints[i]
        # print "---------------------------------"

        # print "frenet cords: ", \
        # if self.first_run:
        # print "road yaw: ", yaw_road


        # move using PID controllers
        # Control vehicle to the right next planned path
        next_p = Pose() # copy.deepcopy(self.target_route_point)
        next_p.position.x = self._waypoint_buffer[0].position.x
        next_p.position.y = self._waypoint_buffer[0].position.y
        #print "type:", type(next_p)
        N = 10
        if path and len(path[opt_ind].x) > 3:
            self.path_buf = [(_x,_y,_yaw) for (_x,_y,_yaw) in zip(path[opt_ind].x[1:],path[opt_ind].y[1:], path[opt_ind].yaw[1:])]
            N = len(self.path_buf) + 1


        # print "path_buf len:", len(self.path_buf)
        # print "path len:", len(path)

        # Additional constraint : if the vehicle almost(right infront) reached the goal, it doesn't follow local planner.

        is_brake = False
        if cur_s > self.wp_s[-6]:
            target_speed = 10
            self.goal_reached = True

        if self.goal_reached:
            target_speed = 0
            is_brake = True

        if len(self.path_buf) > 0 and not self.goal_reached:
            if current_speed > 30 or len(self.path_buf) < 3:
                next_p.position.x = self.path_buf[0][0]
                next_p.position.y = self.path_buf[0][1]
            else:
                next_p.position.x = self.path_buf[2][0]
                next_p.position.y = self.path_buf[2][1]

            np_s, _ = frenet_trajectory.get_frenet_coord(next_p.position.x, next_p.position.y,
                                                          self.total_waypoints)
            # if ((current_pose.position.x - self.path_buf[0][0])**2 \
            #         + (current_pose.position.y - self.path_buf[0][1])**2) < 1.0:
            if np_s - 1.0 <= cur_s :
                #print "pop!!!!!!!!!!!!"
                self.path_buf.pop(0)
            # diff_yaw = np.pi/2.0 - np.arctan2(current_pose.position.y - self.path_buf[5][1],
            #                current_pose.position.x - self.path_buf[5][0])

            # if abs(diff_yaw) > np.pi / 4 and current_speed > 30:
            #     print "Slow down: ", diff_yaw
            #     target_speed = current_speed / (diff_yaw / (np.pi / 8))
        #print "path buf:", self.path_buf
        #self.target_route_point = self._waypoint_buffer[0]

        # default brake point is the front 5 position of current.
        b_x, b_y, _ = frenet_trajectory.get_cartesian(cur_s + 5, cur_d, self.total_waypoints, self.wp_s)

        brake_point = PointStamped()
        brake_point.header.frame_id = "map"
        # brake_point.point.x = self._waypoint_buffer[2].position.x
        # brake_point.point.y = self._waypoint_buffer[2].position.y
        brake_point.point.x = b_x
        brake_point.point.y = b_y
        brake_point.point.z = self._waypoint_buffer[0].position.z

        if len(self.path_buf) > 4:
            brake_point.point.x = self.path_buf[3][0]
            brake_point.point.y = self.path_buf[3][1]
            # target_point.point.x = self.target_route_point.position.x
            # target_point.point.y = self.target_route_point.position.y
            # target_point.point.z = self.target_route_point.position.z

        self.brake_point_publisher.publish(brake_point)
        for ob in obs:
            if (brake_point.point.x - ob[0].x) ** 2 + (brake_point.point.y - ob[0].y) ** 2 <= (CAR_EXTENT + ob[2]) **2:
                print "Brake!!"
                target_speed = 0
                is_brake = True
                break


        # control = self._vehicle_controller.run_step(current_speed, current_pose,
        #                                             self._vehicle_yaw, self.path_buf)


        # print "Vehicle yaw", self._vehicle_yaw
        control = self._vehicle_controller.run_step(
            target_speed, current_speed, current_pose, next_p, self.path_buf, N, self._vehicle_yaw)

        if is_brake:
            control.brake = 1.0
            control.hand_brake = True

        # purge the queue of obsolete waypoints
        # max_index = -1
        # sampling_radius = target_speed * 0.5 / 3.6  # 1 seconds horizon
        # sampling_radius = 30 * 1 / 3.6
        # min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE

        # Pop the waypoint if the ego already pass in s domain.
        first_wp_s, first_wp_d = frenet_trajectory.get_frenet_coord(self._waypoint_buffer[0].position.x,
                                                                    self._waypoint_buffer[0].position.y,
                                                                    self.total_waypoints)

        # print "wp_s", self.wp_s[-2], self.wp_s[-1]
        # print "cur_s", cur_s
        # print "first_wp_s", first_wp_s
        # print "wpbuffer len", len(self._waypoint_buffer)

        if first_wp_s <= cur_s and distance_vehicle(self._waypoint_buffer[0], current_pose.position) < 10:
            if first_wp_s < self.wp_s[-2]:
                self._waypoint_buffer.popleft()

        # for i, route_point in enumerate(self._waypoint_buffer):
        #     if distance_vehicle(
        #             route_point, current_pose.position) < min_distance:
        #         max_index = i
        # if max_index >= 0:
        #     for i in range(max_index + 1):
        #         self._waypoint_buffer.popleft()

        return control, False

    def get_lane_waypoint(self, req):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location()
        carla_position.x = req.x
        carla_position.y = -req.y
        carla_position.z = req.z

        carla_waypoint = self.map.get_waypoint(carla_position)

        response = GetWaypointResponse()
        response.waypoint.pose.position.x = carla_waypoint.transform.location.x
        response.waypoint.pose.position.y = -carla_waypoint.transform.location.y
        response.waypoint.pose.position.z = carla_waypoint.transform.location.z
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        response.waypoint.s = carla_waypoint.s
        response.waypoint.lane_width = carla_waypoint.lane_width
        response.waypoint.lane_change = str(carla_waypoint.lane_change)
        response.waypoint.right_lane_marking.lane_change = str(carla_waypoint.right_lane_marking.lane_change)
        response.waypoint.right_lane_marking.type = str(carla_waypoint.right_lane_marking.type)
        response.waypoint.right_lane_marking.width = carla_waypoint.right_lane_marking.width
        response.waypoint.left_lane_marking.lane_change = str(carla_waypoint.left_lane_marking.lane_change)
        response.waypoint.left_lane_marking.type = str(carla_waypoint.left_lane_marking.type)
        response.waypoint.left_lane_marking.width = carla_waypoint.left_lane_marking.width

        # rospy.logwarn("Get waypoint {}".format(response.waypoint.pose.position))
        return response

    def set_lane_marker(self, lane_node, lane_point, rsp_type):

        # node.pose = Pose(lane_point,Quaternion(0.0,0.0,0.0,1.0))
        if "NONE" in rsp_type:
            lane_node.color.r = 1.0
            lane_node.color.g = 0.0
            lane_node.color.b = 0.0
        elif "Solid" in rsp_type:
            lane_node.color.r = 1.0
            lane_node.color.g = 1.0
            lane_node.color.b = 0.0
        else:
            lane_node.color.r = 1.0
            lane_node.color.g = 1.0
            lane_node.color.b = 1.0
        lane_node.color.a = 0.7

        # lane_node.lifetime = rospy.Duration.from_sec(5)
        # node.frame_locked = False
        # node.ns = "carla_lane"

        # self.tmp_publisher.publish(node)
        # self.lane_id = self.lane_id + 1

# left_wp, right_wp = self.get_coordinate_lanemarking(self.target_route_point.position)
# rsp_wp = self.get_lane_waypoint(
#     carla.Location(x=self.target_route_point.position.x, y=self.target_route_point.position.y,
#                    z=self.target_route_point.position.z)
# )
#
# if left is not None:
#     left_node = self.get_lane_marker(left, rsp.waypoint.left_lane_marking.type)
#     self.left_node_arr.markers.append(left_node)
#     self._lane_left_publisher.publish(self.left_node_arr)
#
# if right is not None:
#     right_node = self.get_lane_marker(right, rsp.waypoint.right_lane_marking.type)
#     self.right_node_arr.markers.append(right_node)
#     self._lane_right_publisher.publish(self.right_node_arr)
#        print("Left: {}, {}; right: {}, {}".format(left.x, left.y, right.x, right.y))