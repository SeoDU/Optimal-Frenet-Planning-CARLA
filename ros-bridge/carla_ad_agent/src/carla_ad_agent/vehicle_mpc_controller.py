#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from carla_msgs.msg import CarlaEgoVehicleControl
from scipy.optimize import minimize
from visualization_msgs.msg import MarkerArray, Marker


class MPCcontroller(object):  # pylint: disable=too-few-public-methods
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """

    def __init__(self):

        self._last_control_time = rospy.get_time()
        self.mpc = ModelPredictiveControl()
        self.state_i = []
        self.u = np.zeros(self.mpc.horizon * 2) # horizon * num_inputs (pedal,steering)

    def run_step(self, current_speed, current_pose, current_yaw, path_buf):
        current_time = rospy.get_time()
        dt = current_time-self._last_control_time
        if dt == 0.0:
            dt = 0.000001
        control = CarlaEgoVehicleControl()

        self.u = np.delete(self.u,0)
        self.u = np.delete(self.u,0)
        if len(self.u) > 2:
            self.u = np.append(self.u, self.u[-2])
            self.u = np.append(self.u, self.u[-2])

        if len(path_buf) > self.mpc.horizon:
            np.hstack([self.u, np.zeros((len(path_buf) - self.mpc.horizon)*2,)])
        else:
            np.resize(self.u,(2*len(path_buf),))

        self.state_i = [current_pose.position.x, current_pose.position.y, current_yaw, current_speed/3.6]
        self.mpc.horizon = len(path_buf)
        self.u.resize((self.mpc.horizon * 2,))

        bounds = []
        for i in range(self.mpc.horizon):
            bounds += [[-10, 10]]  # bounds for pedal
            # bounds += [[-1.0 / ((current_speed+1)/10), 1.0 / ((current_speed+1)/10)]]  # bounds for steering
            bounds += [[-0.5,0.5]]

        u_solution = minimize(self.mpc.cost_function, self.u, (self.state_i, path_buf),
                                method='SLSQP',
                                bounds=bounds,
                                tol = 1e-7)

        self.u = u_solution.x

        self._last_control_time = current_time
        control.throttle = self.u[0]
        control.steer = self.u[1]

        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.1 # -> 10 * 0.05 = 0.5 seconds horizon
        self.mpc_pred_publisher = rospy.Publisher(
            "/pred_traj", Marker, queue_size=1000)
        self.pred_node = self.get_pred_node()
        self.pred_node.color.r = 0.0
        self.pred_node.color.g = 1.0
        self.pred_node.color.b = 0.0
        self.pred_node.color.a = 0.7

        # Reference or set point the controller will achieve.


    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3] # m/s

        a_t = pedal
        steer_angle = steering

        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        v_t_1 = v_t + a_t*dt # - v_t/25.0  -v_t/25 is a rough estimate for friction)
        psi_t_1 = psi_t + v_t*dt*(np.tan(steer_angle)/2.1)

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        ref_len = len(ref)

        self.pred_node.points = []
        first_state = self.plant_model(state, self.dt, u[0], u[1])
        cost += ((first_state[0] - ref[0][0]) ** 2)  # x
        cost += ((first_state[1] - ref[0][1]) ** 2)  # y
        cost += np.tan((first_state[2] - ref[0][2])) ** 2  # y

        self.pred_node.points.append(Point(x=first_state[0],y=first_state[1]))
        for i in range(1,self.horizon):
            # state = [ref[i-1][0], ref[i-1][1], ref[i-1][2], state[3]]
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])

            cost += ((state[0] - ref[i][0])**2) # x
            cost += ((state[1] - ref[i][1])**2) # y
            cost += np.tan((state[2] - ref[i][2]))**2  # angle

            traj_p = Point()
            traj_p.x = state[0]
            traj_p.y = state[1]
            self.pred_node.points.append(traj_p)

        self.mpc_pred_publisher.publish(self.pred_node)


        return cost

    def get_pred_node(self):
        pred_node = Marker()
        pred_node.header.frame_id = "map"

        pred_node.type = Marker.LINE_STRIP  # ARROW : 0, SPHERE : 2
        pred_node.action = Marker.ADD  # ADD
        pred_node.id = 100
        pred_node.header.stamp = rospy.Time.now()
        pred_node.ns = "traj"

        pred_node.pose.orientation.x = 0.0
        pred_node.pose.orientation.y = 0.0
        pred_node.pose.orientation.z = 0.0
        pred_node.pose.orientation.w = 1.0
        pred_node.pose.position.x = 0.0
        pred_node.pose.position.y = 0.0
        pred_node.pose.position.z = 0.0
        pred_node.scale.x = 0.1
        return pred_node