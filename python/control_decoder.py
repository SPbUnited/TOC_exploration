import zmq
from attrs import define
from cattrs import structure, unstructure

import robot_command_model as rcm
import tracker_model as tm

import math
from scipy.optimize import root, least_squares
import numpy as np
from numpy import linalg

from aux import Point

import time as time

context = zmq.Context()
s_tracker = context.socket(zmq.SUB)
s_tracker.connect("ipc:///tmp/transnet.tracker")
s_tracker.setsockopt_string(zmq.SUBSCRIBE, "")

s_control = context.socket(zmq.PUB)
s_control.connect("ipc:///tmp/ether.signals.xsub")

s_draw = context.socket(zmq.PUB)
s_draw.connect("ipc:///tmp/ether.draw.xsub")

s_telemetry = context.socket(zmq.PUB)
s_telemetry.connect("ipc:///tmp/ether.telemetry.xsub")

print("Control decoder example")

last_frame_number = 0

timer1 = time.time()
timer2 = time.time()

elapsed1 = []
elapsed2 = []

@define
class Robot:

    points: list[Point]
    velocities: list[Point]
    current_start: int
    current_goal: int
    EPS = 0.1
    alpha = 0.7
    COORD_EPS = 0.1
    SPEED_EPS = 0.5
    trajectory: list[Point] = []
    trajectory_max_len = 100
    trajectory_update_int = 15
    update_counter: int = 0
    MAX_SPEED = 2
    MAX_ACC = 2

    tsocs_a: list = [1, 1, 1, 1]
    tsocs_T: float = 1

    def update(self, x, y, angle, vel_x, vel_y, vel_angular):
        pos = Point(x, y)
        vel = Point(vel_x, vel_y)

        err = self.alpha*(pos - self.points[self.current_goal]).mag() + (1 - self.alpha)*(vel - self.velocities[self.current_goal]).mag() 
        print(err)
        if err < self.EPS:
            self.current_start = self.current_goal
            self.current_goal = (self.current_goal + 1)%len(self.points)

        self.update_counter += 1
        if self.update_counter > self.trajectory_update_int:
            self.update_counter = 0
            self.trajectory.append(pos)
            if len(self.trajectory) > self.trajectory_max_len:
                self.trajectory.pop(0)

        # self.points = [Point(1, 0.5*math.sin(time.time()))]

        draw_points_data = {
            "points": {
                "data": [ 
                    {
                        "type": "circle",
                        "x": p.x*1000,
                        "y": -p.y*1000,
                        "radius": 50,
                        "color": "#0000FF" if p == self.points[self.current_goal] else "#FF0000",
                    } for p in self.points],
                "is_visible": True,
            }
        }
        draw_speeds_data = {
            "speeds": {
                "data": [
                    {
                        "type": "arrow",
                        "x": z[0].x*1000,
                        "y": -z[0].y*1000,
                        "dx": z[1].x*1000,
                        "dy": -z[1].y*1000,
                        "color": "#0000FF" if z[0] == self.points[self.current_goal] else "#FF0000",
                        "width": 20,
                    }  for z in zip(self.points, self.velocities)],
                "is_visible": True,
            }
        }
        s_draw.send_json(draw_points_data)
        s_draw.send_json(draw_speeds_data)
        draw_trajectory_data = {
                "trajectory": {
                    "data": [
                        {
                            "type": "line",
                            "x_list": [p.x*1000 for p in self.trajectory],
                            "y_list": [-p.y*1000 for p in self.trajectory],
                            "color": "#888888",
                            "width": 10,
                        },
                    ],
                    "is_visible": True,
                }
            }
        s_draw.send_json(draw_trajectory_data)

        time0 = time.time()
        result_tsocs = self.tsocs(pos, self.points[self.current_goal], vel, self.velocities[self.current_goal], angle)
        time1 = time.time()
        
        print("TSOCS time = " + str((time1-time0)*1000))
        elapsed1.append((time1-time0)*1000)
        if len(elapsed1) > 1000: 
            elapsed1.pop(0)

        time0 = time.time()
        result_bangbang = self.bangbang(pos, self.points[self.current_goal], vel, self.velocities[self.current_goal], angle)
        time1 = time.time()
        print("bangbang time = " + str((time1-time0)*1000))
        elapsed2.append((time1-time0)*1000)
        if len(elapsed2) > 1000: 
            elapsed2.pop(0)

        data = {"Computation time": 
                " TSOCS avg time:     {0:.3f}ms (min: {1:.3f}ms, max: {2:.3f}ms)\n".format(np.average(elapsed1), min(elapsed1), max(elapsed1)) +
                " Bang-bang avg time: {0:.3f}ms (min: {1:.3f}ms, max: {2:.3f}ms)\n".format(np.average(elapsed2), min(elapsed2), max(elapsed2))
                }
        s_telemetry.send_json(data)

        return result_bangbang
        


    def bangbang(self, pos, goal, vel, vel_goal, angle):
        v_max = self.max_vel_bang_bang(pos, goal, vel, vel_goal)
        T = self.get_bang_bang_time(pos, goal, v_max, vel, vel_goal)
        
        plan = []
        for t in range(0, 21, 1):
            values = self.get_bang_bang_values(pos, goal, v_max, T/20*t, vel, vel_goal)
            plan.append(values[2])

        draw_planned_data = {
                "plan": {
                    "data": [
                        {
                            "type": "line",
                            "x_list": [p.x*1000 for p in plan],
                            "y_list": [-p.y*1000 for p in plan],
                            "color": "#FF00FF",
                            "width": 10,
                        },
                    ],
                    "is_visible": True,
                }
            }
        s_draw.send_json(draw_planned_data)

        global timer1
        timer1 = time.time()
        values = self.get_bang_bang_values(pos, goal, v_max, max(timer1-time.time(), 0.1), vel, vel_goal)
        return values[1].y, values[1].x, -angle*2
    
    def short_dist(self, v_max: np.ndarray, args: list) -> np.ndarray:
        """Поиск короткого решения"""
        v_start = args[0]
        v_end = args[1]
        delta_r = args[2]
        a_max = args[3]
        return (
            2 * a_max * delta_r
            - (v_max + v_start) * np.linalg.norm(v_max - v_start)
            - (v_max + v_end) * np.linalg.norm(v_max - v_end)
        )


    def long_dist(self, ang: np.ndarray, args: list) -> np.ndarray:
        """Поиск длинного решения"""
        max_speed = args[0]
        v_start = args[1]
        v_end = args[2]
        delta_r = args[3]
        a_max = args[4]
        v_max = np.array([max_speed * np.cos(ang[0]), max_speed * np.sin(ang[0])])
        val = (
            2 * a_max * np.array(delta_r)
            - (v_max + v_start) * np.linalg.norm(v_max - v_start)
            - (v_max + v_end) * np.linalg.norm(v_max - v_end)
        )
        return np.array([(val[0] * v_max[0] + val[1] * v_max[1]) / np.linalg.norm(val) / np.linalg.norm(v_max) - 1])

    def max_vel_bang_bang(
        self,
        r_start: Point,
        r_end: Point,
        v_start: Point = Point(0, 0),
        v_end: Point = Point(0, 0),
        max_speed: float = MAX_SPEED,
        a_max: float = MAX_ACC,
        ) -> Point:
        """Найти макс. скорость бенг-бенга, однозначно его задает"""
        delta_r = r_end - r_start
        v_max_initial = np.array([0, 0])
        if delta_r.mag() != 0:
            v_max_initial = np.array([delta_r.x, delta_r.y]) / delta_r.mag() * max_speed
        args = [np.array([v_start.x, v_start.y]), np.array([v_end.x, v_end.y]), np.array([delta_r.x, delta_r.y]), a_max]
        short_solution = root(self.short_dist, v_max_initial, args=args)

        if not short_solution.success:
            short_solution = least_squares(self.short_dist, v_max_initial, method="lm", args=[args])

        v_max = Point(short_solution.x[0], short_solution.x[1])

        if v_max.mag() > max_speed:
            args.insert(0, max_speed)
            ang_initial = np.array([delta_r.arg()])
            long_solution = root(self.long_dist, ang_initial, args=args)
            if not long_solution.success:
                long_solution = least_squares(self.long_dist, ang_initial, method="lm", args=[args])
            angle = long_solution.x[0]
            v_max = Point(max_speed * np.cos(angle), max_speed * np.sin(angle))

        return v_max

    def get_bang_bang_time(
        self,
        r_start: Point,
        r_end: Point,
        v_max: Point,
        v_start: Point = Point(0, 0),
        v_end: Point = Point(0, 0),
        max_speed: float = MAX_SPEED,
        a_max: float = MAX_ACC,
        ) -> float:
        """Найти время проезда по бенг-бенгу"""
        return (
            (v_max - v_start).mag() / a_max
            + (
                r_end
                - r_start
                - ((v_max + v_start) * (v_max - v_start).mag() + (v_end + v_max) * (v_end - v_max).mag()) / 2 / a_max
            ).mag()
            / max_speed
            + (v_end - v_max).mag() / a_max
        )

    def get_bang_bang_values(
        self,
        r_start: Point,
        r_end: Point,
        v_max: Point,
        t: float,
        v_start: Point = Point(0, 0),
        v_end: Point = Point(0, 0),
        max_speed: float = MAX_SPEED,
        a_max: float = MAX_ACC,
    ) -> list[Point]:
        """Возвращает кинематические величины в любой момент проезда по бенг-бенгу"""
        p1 = r_start + (v_max + v_start) * (v_max - v_start).mag() / 2 / a_max
        p2 = r_end - (v_end + v_max) * (v_end - v_max).mag() / 2 / a_max
        t1 = (v_max - v_start).mag() / a_max
        t2 = t1 + (p2 - p1).mag() / max_speed
        t3 = t2 + (v_end - v_max).mag() / a_max
        values: list[Point] = []
        if t < 0:
            values.append((v_max - v_start).unity() * a_max)
            values.append(v_start)
            values.append(r_start)
        if t < t1:
            values.append((v_max - v_start).unity() * a_max)
            values.append(v_start + values[0] * t)
            values.append(r_start + v_start * t + values[0] * t**2 / 2)
        elif t < t2:
            values.append(Point(0, 0))
            values.append(v_max)
            values.append(p1 + v_max * (t - t1))
        elif t < t3:
            values.append((v_end - v_max).unity() * a_max)
            values.append(v_max + values[0] * (t - t2))
            values.append(p2 + v_max * (t - t2) + values[0] * (t - t2) ** 2 / 2)
        else:
            values.append((v_end - v_max).unity() * a_max)
            values.append(v_end)
            values.append(r_end)
        return values


    def tsocs(self, pI, pF, vI, vF, angle):
        pI_np = np.array([pI.x, pI.y])
        pF_np = np.array([pF.x, pF.y])
        vI_np = np.array([vI.x, vI.y])
        vF_np = np.array([vF.x, vF.y])
        Umax = self.MAX_ACC

        global timer2
        dT = timer2 - time.time()
        timer2 = time.time()

        self.tsocs_a[2] += self.tsocs_a[0]*dT
        self.tsocs_a[3] += self.tsocs_a[1]*dT
        self.tsocs_T -= dT


        plan_1 = []
        for t in range(0, 21, 1):
            values = self.get_tocorba_vel_pos(pI_np, vI_np, self.tsocs_a, Umax, self.tsocs_T/20*t)
            plan_1.append(values)
        draw_tocorba_speeds = {
            "tocorba plan 1 speeds": {
                "data": [
                    {
                        "type": "line",
                        "x_list": [p[1][0]*1000, (p[1][0]+p[0][0])*1000],
                        "y_list": [-p[1][1]*1000, -(p[1][1]+p[0][1])*1000],
                        "color": "#00FFFF",
                        "width": 5,
                    } for p in plan_1],
                "is_visible": True,
            }
        }
        s_draw.send_json(draw_tocorba_speeds)
        draw_tocorba_plan = {
            "tocorba plan 1": {
                "data": [
                    {
                        "type": "line",
                        "x_list": [p[1][0]*1000 for p in plan_1],
                        "y_list": [-p[1][1]*1000 for p in plan_1],
                        "color": "#0000FF",
                        "width": 10,
                    },
                ],
                "is_visible": True,
            }
        }
        s_draw.send_json(draw_tocorba_plan)


        success_2, a_2, T = self.tsocs_stage_2(self.tsocs_a, pI_np, pF_np, vI_np, vF_np, Umax, self.tsocs_T)
        if success_2:
            self.tsocs_a = a_2
            self.tsocs_T = T
        else:
            success_1, a_0, a_1, Tmax = self.tsocs_stage_1(pI_np, pF_np, vI_np, vF_np, Umax)
            success_2, a_2, T = self.tsocs_stage_2(a_1, pI_np, pF_np, vI_np, vF_np, Umax, Tmax)


            plan_0 = []
            for t in range(0, 21, 1):
                values = self.get_tocorba_vel_pos(pI_np, vI_np, a_0, Umax, Tmax/20*t)
                plan_0.append(values[1])
            draw_tocorba_plan = {
                "tocorba plan 0": {
                    "data": [
                        {
                            "type": "line",
                            "x_list": [p[0]*1000 for p in plan_0],
                            "y_list": [-p[1]*1000 for p in plan_0],
                            "color": "#FFFFFF",
                            "width": 10,
                        },
                    ],
                    "is_visible": True,
                }
            }
            s_draw.send_json(draw_tocorba_plan)

            if success_2:
                self.tsocs_a = a_2
                self.tsocs_T = T
        

        max_speed = 0
        plan_2 = []
        for t in range(0, 21, 1):
            values = self.get_tocorba_vel_pos(pI_np, vI_np, self.tsocs_a, Umax, self.tsocs_T/20*t)
            plan_2.append(values)
            if linalg.norm(values[0]) > max_speed:
                max_speed = linalg.norm(values[0])

        draw_tocorba_speeds = {
            "tocorba plan 2 speeds": {
                "data": [
                    {
                        "type": "line",
                        "x_list": [p[1][0]*1000, (p[1][0]+p[0][0])*1000],
                        "y_list": [-p[1][1]*1000, -(p[1][1]+p[0][1])*1000],
                        "color": "#FFFF00",
                        "width": 5,
                    } for p in plan_2],
                "is_visible": True,
            }
        }
        s_draw.send_json(draw_tocorba_speeds)
        draw_tocorba_plan = {
            "tocorba plan 2": {
                "data": [
                    {
                        "type": "line",
                        "x_list": [p[1][0]*1000 for p in plan_2],
                        "y_list": [-p[1][1]*1000 for p in plan_2],
                        "color": "#FFAA00",
                        "width": 10,
                    },
                ],
                "is_visible": True,
            }
        }
        s_draw.send_json(draw_tocorba_plan)

        vT, xT = self.get_tocorba_vel_pos(pI_np, vI_np, self.tsocs_a, Umax, max(dT, 0.1))
        return vT[1], vT[0], -angle*2

    def tsocs_stage_1(self, pI, pF, vI, vF, Umax):
        theta = math.atan2(pF[1] - pI[1], pF[0] - pI[0])
        T, ttX, signX = self.calc_1DOF_optimal_times_2(pI[0], pF[0], vI[0], vF[0], Umax)
        T, ttY, signY = self.calc_1DOF_optimal_times_2(pI[1], pF[1], vI[1], vF[1], Umax)
        a_0 = [1, -2, 3, -4]
        a_0[2] = 1*math.cos(theta)
        a_0[3] = 1*math.sin(theta)
        a_0[0] = -a_0[2]/ttX
        a_0[1] = -a_0[3]/ttY
        s1 = linalg.norm(pF - pI)/Umax - linalg.norm(vI**2 + vF**2)/(2*Umax**2)
        if s1 <= 0:
            s2 = 0
        else:
            s2 = 2*math.sqrt(s1)
        Tmax = (linalg.norm(vI + vF))/Umax + s2

        args = [pI, pF, vI, vF, Umax, Tmax]
        solution = least_squares(
            self.tocorba_calc_cost_1, 
            a_0, 
            method="trf", 
            max_nfev=50, 
            ftol=1e-2, 
            xtol=1e-2, 
            gtol=1e-2, 
            args=args,
            )
        return solution.success, a_0, solution.x, Tmax
    
    def tsocs_stage_2(self, a_1, pI, pF, vI, vF, Umax, Tmax):
        a_init = [a_1[0], a_1[1], a_1[2], a_1[3], Tmax]
        args = [pI, pF, vI, vF, Umax]
        solution = least_squares(
            self.tocorba_calc_cost_2, 
            a_init, 
            method="trf", 
            max_nfev=15, 
            ftol=1e-5, 
            xtol=1e-5, 
            gtol=1e-5, 
            args=args, 
            # bounds=(
            #     [-np.inf, -np.inf, -np.inf, -np.inf, 0], 
            #     [np.inf, np.inf, np.inf, np.inf, Tmax]
            #     ),
            )
        a_2 = [
            solution.x[0], 
            solution.x[1], 
            solution.x[2], 
            solution.x[3], 
        ]
        T = solution.x[4]
        return solution.success, a_2, T

    def tocorba_calc_cost_2(self, a, xI, xF, vI, vF, Umax):
        # print("CALC COST_2")
        cost = self.tocorba_calc_cost_1(a, xI, xF, vI, vF, Umax, a[4])
        return [cost[0], cost[1], cost[2], cost[3], 0]


    def tocorba_calc_cost_1(self, a, xI, xF, vI, vF, Umax, T):
        # print("CALC COST_1")
        vT, xT = self.get_tocorba_vel_pos(xI, vI, a, Umax, T)
        return [
            math.pow(xF[0] - xT[0], 2),
            math.pow(xF[1] - xT[1], 2),
            math.pow(vF[0] - vT[0], 2),
            math.pow(vF[1] - vT[1], 2),
            ]
      

    def calc_1DOF_optimal_times_2(self, xI, xF, vI, vF, Umax):
        # print("CALC 1_DOF")
        try:
            sign = 1
            tt = -(2*vI - math.sqrt(2)*math.sqrt(vF**2 + vI**2 + 2*Umax*xF - 2*Umax*xI))/(2*Umax)
            T = (vI - vF + 2*Umax*tt)/Umax
            if tt > T:
                raise Exception()
        except:    
            sign = -1
            tt = (2*vI + math.sqrt(2)*math.sqrt(vF**2 + vI**2 - 2*Umax*xF + 2*Umax*xI))/(2*Umax)
            T = (vF - vI + 2*Umax*tt)/Umax
        
        return T, tt, sign


    def get_tocorba_vel_pos(self, xI, vI, a, Umax, t):
        # print("CALC VELOCITY AND POSITION")
        a1 = a[0]
        a2 = a[1]
        a3 = a[2]
        a4 = a[3]

        s1 = a1*a1 + a2*a2
        s13 = a3*a3 + a4*a4
        s10 = a1*a3 + a2*a4
        s2 = 2*t*s10 + s13
        s7 = math.sqrt(t*t*s1 + s2)
        s25 = math.sqrt(s1)
        s26 = math.sqrt(s13)
        kEpsilonSq = 1e-18
        s30 = (s25*s7 + t*s1 + s10 + kEpsilonSq) / (s25*s26 + s10 + kEpsilonSq)
        if s30 <= 0:
            s24 = -1e20
        else:
            s24 = math.log(s30)
        s27 = s25*s10
        s28 = s25*s25*s25
        s29 = s28*s25*s25
        
        vtx, xtx = self.calc_tocorba_vel_pos(xI[0], vI[0], Umax, t, a1, a2, a3, a4, s1, s7, s10, s24, s25, s26, s27, s28, s29)
        vty, xty = self.calc_tocorba_vel_pos(xI[1], vI[1], Umax, t, a2, a1, a4, a3, s1, s7, s10, s24, s25, s26, s27, s28, s29)

        return np.array([vtx, vty]), np.array([xtx, xty])


    def calc_tocorba_vel_pos(self, xI, vI, Umax, t, a1, a2, a3, a4, s1, s7, s10, s24, s25, s26, s27, s28, s29):
        s6 = a2*a3 - a1*a4
        s21 = a2*s6

        v = vI + Umax*( a1*(s7 - s26)/s1 + s21*s24/(s1*s25) )

        x = xI + vI*t + Umax*( a1*(s7*(s27 + t*s28) + s6*s6*s24 \
        - s26*(s27 + 2*t*s28))/(2*s29) \
        + s21*(s24*(s1*t + s10) - s25*(s7 - s26))/s29 )

        return v, x


robot = Robot(points = [Point(-1.5, -0.12), Point(-1, 0.12), Point(-0.5, -0.12), Point(0, 0.12), Point(1, 0), Point(0, 1), Point(-3, 2)],
              velocities = [Point(0, 0), Point(0.2, 0), Point(0.2, 0), Point(0.2, 0), Point(0.3, 0.4), Point(0.3, 0.4), Point(-0.5, -0.2)],
              current_start = 0,
              current_goal = 1)

# robot = Robot(points = [Point(0, 0), Point(1, 0.25*math.sin(time.time()))],
#               velocities = [Point(0, 0), Point(0, 0)],
#               current_start = 0,
#               current_goal = 1)

while True:
    print(".")
    tracker_data = structure(s_tracker.recv_json(), tm.TrackerWrapperPacket)
    while True:
        try:
            new_tracker_data = structure(s_tracker.recv_json(flags=zmq.NOBLOCK), tm.TrackerWrapperPacket)
            tracker_data = new_tracker_data
        except:
            break
    # print(tracker_data.tracked_frame.timestamp)
    # print(tracker_data.tracked_frame.frame_number)
    # print(tracker_data.tracked_frame.frame_number - last_frame_number)
    # if len(tracker_data.tracked_frame.robots) > 0: print(tracker_data.tracked_frame.robots[0])

    last_frame_number = tracker_data.tracked_frame.frame_number

    tracked_robot = None

    for r in tracker_data.tracked_frame.robots:
        if r.robot_id.team == tm.Team.YELLOW and r.robot_id.id == 0:
            tracked_robot = r

    if tracked_robot is None:
        continue

    req_vel_x, req_vel_y, req_vel_w = robot.update(
        tracked_robot.pos.x,
        tracked_robot.pos.y,
        tracked_robot.orientation,
        tracked_robot.vel.x,
        tracked_robot.vel.y,
        tracked_robot.vel_angular,
    )

    control_data = rcm.RobotControlExt(
        isteamyellow=True,
        robot_commands=[
            rcm.RobotCommand(
                id=0,
                move_command=rcm.RobotMoveCommand(
                    # wheel_velocity=rcm.MoveWheelVelocity(
                    #     front_right=0,
                    #     back_right=0,
                    #     back_left=0,
                    #     front_left=1,
                    # )
                    local_velocity=rcm.MoveLocalVelocity(
                        forward=req_vel_y,
                        left=req_vel_x,
                        angular=req_vel_w,
                    ),
                    #global_velocity=rcm.MoveGlobalVelocity(
                    #    x=req_vel_x,
                    #    y=req_vel_y,
                    #    angular=req_vel_w,
                    #),
                ),
                kick_speed=0,
                kick_angle=0,
                dribbler_speed=0,
            ),
        ],
    )
    s_control.send_json(
        {"transnet": "actuate_robot", "data": unstructure(control_data)}
    )
