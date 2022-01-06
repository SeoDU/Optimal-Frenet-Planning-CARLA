import numpy as np
from geometry_msgs.msg import Point,PointStamped
from visualization_msgs.msg import MarkerArray,Marker
import math
from collections import deque
from copy import deepcopy

V_MAX = 110     # maximum velocity [m/s]
ACC_MAX = 20000  # maximum acceleration [m/ss]
K_MAX = 30     # maximum curvature [1/m]

MIN_T = 0.2 # minimum terminal time [s] // remainder Must be 0
MAX_T = 0.4 # maximum terminal time [s]
DT_T = 0.04 # dt for terminal time [s]
DT = 0.04

K_J = 0.1 # weight for jerk
K_T = 0.1 # weight for terminal time
K_D = 0.1 # weight for consistency
K_V = 1.0 # weight for getting to target speed
K_LAT = 1.0 # weight for lateral direction
K_LON = 1.0 # weight for longitudinal direction
K_Di = 0 # or 20000
K_Obs = 1e-3

CAR_EXTENT = 2.3 # collision check distance [m]

class QuinticPolynomial:

    def __init__(self, xi, vi, ai, xf, vf, af, T):
        # calculate coefficient of quintic polynomial
        # used for lateral trajectory
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5*ai

        A = np.array([[T**3, T**4, T**5],
                      [3*T**2, 4*T**3, 5*T** 4],
                      [6*T, 12*T**2, 20*T**3]])
        b = np.array([xf - self.a0 - self.a1*T - self.a2*T**2,
                      vf - self.a1 - 2*self.a2*T,
                      af - 2*self.a2])

        #print A.dtype, b.dtype
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    # calculate postition info.
    def calc_pos(self, t):
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5 * t ** 5
        return x

    # calculate velocity info.
    def calc_vel(self, t):
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
        return v

    # calculate acceleration info.
    def calc_acc(self, t):
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3
        return a

    # calculate jerk info.
    def calc_jerk(self, t):
        j = 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2
        return j


class QuarticPolynomial:

    def __init__(self, xi, vi, ai, vf, af, T):
        # calculate coefficient of quartic polynomial
        # used for longitudinal trajectory
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5*ai

        A = np.array([[3*T**2, 4*T**3],
                             [6*T, 12*T**2]])
        b = np.array([vf - self.a1 - 2*self.a2*T,
                             af - 2*self.a2])

        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    # calculate postition info.
    def calc_pos(self, t):
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4
        return x

    # calculate velocity info.
    def calc_vel(self, t):
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3
        return v

    # calculate acceleration info.
    def calc_acc(self, t):
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2
        return a

    # calculate jerk info.
    def calc_jerk(self, t):
        j = 6*self.a3 + 24*self.a4*t
        return j

class FrenetPath:

    def __init__(self):
        # time
        self.t = []

        # lateral traj in Frenet frame
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []

        # longitudinal traj in Frenet frame
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []

        # cost
        self.c_lat = 0.0
        self.c_lon = 0.0
        self.c_tot = 0.0

        # combined traj in global frame
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.kappa = []

def get_closest_waypoints(x, y, total_wps):
    min_len = 1e10
    closest_wp = 0

    for i in range(len(total_wps)):
        dist = get_dist(x, y, total_wps[i][0], total_wps[i][1])

        if dist < min_len:
            min_len = dist
            closest_wp = i

    return closest_wp

def next_waypoint(x, y, total_wps):
    closest_wp = get_closest_waypoints(x, y, total_wps)
    if closest_wp + 1 == len(total_wps):
        return None

    map_vec = [total_wps[closest_wp + 1][0] - total_wps[closest_wp][0],
               total_wps[closest_wp + 1][1] - total_wps[closest_wp][1]]
    ego_vec = [x - total_wps[closest_wp][0], y - total_wps[closest_wp][1]]

    direction  = np.sign(np.dot(map_vec, ego_vec))

    if direction >= 0:
        next_wp = closest_wp + 1
    else:
        next_wp = closest_wp

    return next_wp

def get_dist(x, y, _x, _y):
    return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet_coord(x,y,total_wps):
    next_wp = next_waypoint(x,y,total_wps)
    if not next_wp:
        return 0,0
    prev_wp = next_wp - 1

    n_x = total_wps[next_wp][0] - total_wps[prev_wp][0]
    n_y = total_wps[next_wp][1] - total_wps[prev_wp][1]
    x_x = x - total_wps[prev_wp][0]
    x_y = y - total_wps[prev_wp][1]

    # print "next/prev wps:", next_wp, " ", prev_wp
    # print "wp_next/wp_prev: ", total_wps[next_wp], total_wps[prev_wp]
    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
    proj_x = proj_norm*n_x
    proj_y = proj_norm*n_y

    #-------- get frenet d
    frenet_d = get_dist(x_x,x_y,proj_x,proj_y)

    ego_vec = [x-total_wps[prev_wp][0], y-total_wps[prev_wp][1], 0]
    map_vec = [n_x, n_y, 0]
    d_cross = np.cross(ego_vec,map_vec)
    if d_cross[-1] > 0:
        frenet_d = -frenet_d

    #-------- get frenet s
    frenet_s = 0
    for i in range(prev_wp):
        frenet_s = frenet_s + get_dist(total_wps[i][0],total_wps[i][1],total_wps[i+1][0],total_wps[i+1][1]);

    frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y)

    return frenet_s, frenet_d

def get_cartesian(s,d,total_wps,wp_s):
    prev_wp = 0
    s = np.mod(s, wp_s[-1]) # EDITED
    while (s > wp_s[prev_wp + 1]) and (prev_wp < len(wp_s) - 2):
        prev_wp = prev_wp + 1

    next_wp = np.mod(prev_wp + 1, len(total_wps))
    dx = (total_wps[next_wp][0] - total_wps[prev_wp][0])
    dy = (total_wps[next_wp][1] - total_wps[prev_wp][1])

    heading = np.arctan2(dy, dx)

    seg_s = s - wp_s[prev_wp];

    seg_x = total_wps[prev_wp][0] + seg_s*np.cos(heading);
    seg_y = total_wps[prev_wp][1] + seg_s*np.sin(heading);

    perp_heading = heading + 90 * np.pi/180;
    x = seg_x + d*np.cos(perp_heading);
    y = seg_y + d*np.sin(perp_heading);

    return x,y,heading

def calc_frenet_paths(TARGET_SPEED, DF_SET, si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, obs):
    frenet_paths = []

    for df in DF_SET:
        for T in np.arange(MIN_T, MAX_T + DT_T, DT_T):
            fp = FrenetPath()
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

            fp.t = [t for t in np.arange(0.0, T, DT)]
            fp.d = [lat_traj.calc_pos(t) for t in fp.t]
            fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
            fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
            fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

            # Longitudinal motion planning (velocity keeping)
            tfp = deepcopy(fp)
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

            tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
            tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
            tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
            tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

            # (In case T < MAX_T)
            for _t in np.arange(T, MAX_T, DT):
                tfp.t.append(_t)
                tfp.d.append(tfp.d[-1])
                _s = tfp.s[-1] + tfp.s_d[-1] * DT
                tfp.s.append(_s)

                tfp.s_d.append(tfp.s_d[-1])
                tfp.s_dd.append(tfp.s_dd[-1])
                tfp.s_ddd.append(tfp.s_ddd[-1])

                tfp.d_d.append(tfp.d_d[-1])
                tfp.d_dd.append(tfp.d_dd[-1])
                tfp.d_ddd.append(tfp.d_ddd[-1])

            J_lat = sum(np.power(tfp.d_ddd, 2))  # lateral jerk
            J_lon = sum(np.power(tfp.s_ddd, 2))  # longitudinal jerk

            # cost for consistency
            d_diff = (tfp.d[-1] - opt_d) ** 2
            # cost for target speed
            v_diff = (TARGET_SPEED - tfp.s_d[-1]) ** 2

            # lateral cost
            tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff
            # logitudinal cost
            tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff

            obs_cost = 0
            for ob in obs:
                 obs_cost += np.sum([((_x - ob[0].x) ** 2 + (_y - ob[0].y) ** 2)
                     for (_x, _y) in zip(tfp.x, tfp.y)])

            # total cost combined
            tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon \
                        + K_Di * abs(tfp.d[-1]) + K_Obs / (1 + obs_cost)

            frenet_paths.append(tfp)

    return frenet_paths


def frenet_optimal_planning(TARGET_SPEED, DF_SET, si, si_d, si_dd,
                            sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, total_wpts, wp_s, opt_d):
    fplist = calc_frenet_paths(TARGET_SPEED, DF_SET, si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, obs)
    fplist = calc_global_paths(fplist, total_wpts, wp_s)
    fplist = check_path(fplist, obs, total_wpts, wp_s)
    # find minimum cost path
    min_cost = float("inf")
    opt_traj = None
    opt_ind = 0
    _opt_ind = 0
    for fp in fplist:
        if min_cost >= fp.c_tot:
            min_cost = fp.c_tot
            opt_traj = fp
            _opt_ind = opt_ind
        opt_ind += 1

    try:
        _opt_ind
    except NameError:
        print(" No solution ! ")

    return fplist, _opt_ind

def collision_check(fp, obs, total_wpts, wp_s):
    for ob in obs:
        d = [ ((_x - ob[0].x) **2 + (_y - ob[0].y) **2)
              for (_x, _y) in zip(fp.x, fp.y)]
        d_mv = [ ((_x - (ob[0].x + ob[3] * 3.0) ) **2 + (_y -  (ob[0].y + ob[4] * 3.0) ) **2)
              for (_x, _y) in zip(fp.x, fp.y)]

        collision = any([di <= (CAR_EXTENT + ob[2])**2 for di in d])
        collision_mv = any([di <= (CAR_EXTENT + ob[2]) ** 2 for di in d_mv])
        if collision or collision_mv:
            return True


    # for i in range(len(obs[:, 0])):
    #     # get obstacle's position (x,y)
    #     obs_xy = get_cartesian( obs[i, 0], obs[i, 1], total_wpts, wp_s)
    #
    #     d = [((_x - obs_xy[0]) ** 2 + (_y - obs_xy[1]) ** 2)
    #          for (_x, _y) in zip(fp.x, fp.y)]
    #
    #     collision = any([di <= COL_CHECK ** 2 for di in d])
    #
    #     if collision:
    #         return True

    return False

def check_path(fplist, obs, total_wpts, wp_s):
    ok_ind = []
    for i, _path in enumerate(fplist):
        acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
        if any([v > V_MAX for v in _path.s_d]):  # Max speed check
            print "MAX speed EXCESS", v
            continue
        elif any([acc > ACC_MAX**2 for acc in acc_squared]):
            print "MAX accel EXCESS", acc
            continue
        elif any([abs(kappa) > K_MAX for kappa in fplist[i].kappa]):  # Max curvature check
            print "MAX kappa EXCESS", kappa
            continue
        elif collision_check(_path, obs, total_wpts, wp_s):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def calc_global_paths(fplist, total_wpts, wp_s):

    # transform trajectory from Frenet to Global
    for fp in fplist:
        for i in range(len(fp.s)):
            _s = fp.s[i]
            _d = fp.d[i]
            _x, _y, _ = get_cartesian(_s, _d, total_wpts, wp_s)
            fp.x.append(_x)
            fp.y.append(_y)

        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(np.arctan2(dy, dx))
            fp.ds.append(np.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            fp.kappa.append(yaw_diff / fp.ds[i])

    return fplist