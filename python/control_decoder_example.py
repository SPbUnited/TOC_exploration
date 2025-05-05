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

#import time

context = zmq.Context()
s_tracker = context.socket(zmq.SUB)
s_tracker.connect("ipc:///tmp/transnet.tracker")
s_tracker.setsockopt_string(zmq.SUBSCRIBE, "")

s_control = context.socket(zmq.PUB)
s_control.connect("ipc:///tmp/ether.signals.xsub")

s_draw = context.socket(zmq.PUB)
s_draw.connect("ipc:///tmp/ether.draw.xsub")

print("Control decoder example")

last_frame_number = 0

time = 0

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
    MAX_ACC = 0.2

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

        self.tocorba(pos, self.points[self.current_goal], vel, self.velocities[self.current_goal], angle)
        return self.bangbang(pos, self.points[self.current_goal], vel, self.velocities[self.current_goal], angle)
        


    def bangbang(self, pos, goal, vel, vel_goal, angle):
        v_max = self.max_vel_bang_bang(pos, goal, vel, vel_goal)
        T = self.get_bang_bang_time(pos, goal, v_max, vel, vel_goal)
        print(T)
        
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

        values = self.get_bang_bang_values(pos, goal, v_max, 0.07, vel, vel_goal)
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


    def tocorba(self, pI, pF, vI, vF, angle):
        dP = pF - pI
        # dP_np = np.array([dP.x, dP.y])
        pI_np = np.array([pI.x, pI.y])
        pF_np = np.array([pF.x, pF.y])
        vI_np = np.array([vI.x, vI.y])
        vF_np = np.array([vF.x, vF.y])

        # vI_proj = np.dot(vI_np, dP_np) / np.dot(dP_np, dP_np)
        # vF_proj = np.dot(vF_np, dP_np) / np.dot(dP_np, dP_np)

        theta = math.atan2(dP.y, dP.x)

        Umax = self.MAX_ACC

        # T, tt = self.calc_1DOF_optimal_times(0, linalg.norm(dP_np), vI_proj, vF_proj, Umax)

        a_init = [1, -2, 3, -4]
        # a_init.append(1 / (math.cos(theta)*(Umax*(T - tt) + vF_proj)))
        # a_init.append(-a_init[0]*tt)
        # a_init.append(1 / (math.sin(theta)*(Umax*(T - tt) + vF_proj)))
        # a_init.append(-a_init[2]*tt)

        T, ttX, signX = self.calc_1DOF_optimal_times_2(pI.x, pF.x, vI.x, vF.x, Umax)
        T, ttY, signY = self.calc_1DOF_optimal_times_2(pI.y, pF.y, vI.y, vF.y, Umax)
        a_init[2] = signX*math.cos(theta)
        a_init[3] = signY*math.sin(theta)
        a_init[0] = -a_init[2]/ttX
        a_init[1] = -a_init[3]/ttY

        Tmax = (vI + vF).mag()/Umax \
        + 2*math.sqrt((pF - pI).mag()/Umax - (vI**2 + vF**2).mag()/(2*Umax**2))

        # plan_0 = []
        # for t in range(0, 41, 1):
        #     values = self.get_tocorba_vel_pos(pI_np, vI_np, a_init, Umax, Tmax/40*t)
        #     plan_0.append(values[1])
        # draw_tocorba_plan = {
        #     "tocorba plan 0": {
        #         "data": [
        #             {
        #                 "type": "line",
        #                 "x_list": [p[0]*1000 for p in plan_0],
        #                 "y_list": [-p[1]*1000 for p in plan_0],
        #                 "color": "#FFFF00",
        #                 "width": 10,
        #             },
        #         ],
        #         "is_visible": True,
        #     }
        # }
        # s_draw.send_json(draw_tocorba_plan)

        print(pI)
        print(pF)
        print(vI)
        print(vF)
        # print(Tmax)
        # print(a_init)

        args_1 = [pI_np, pF_np, vI_np, vF_np, Umax, Tmax]
        solution = least_squares(self.tocorba_calc_cost_1, a_init, method="trf", max_nfev=50, args=args_1)

        # print(solution)
        a_1 = solution.x
        vT, xT = self.get_tocorba_vel_pos(pI_np, vI_np, a_1, Umax, Tmax)
        print(xT, vT, Tmax)

        plan_1 = []
        for t in range(0, 41, 1):
            values = self.get_tocorba_vel_pos(pI_np, vI_np, a_1, Umax, Tmax/40*t)
            plan_1.append(values)
            # print(values[1], values[0], Tmax/40*t)
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

        a_init_2 = [a_1[0], a_1[1], a_1[2], a_1[3], Tmax]
        args_2 = [pI_np, pF_np, vI_np, vF_np, Umax]
        solution = least_squares(self.tocorba_calc_cost_2, a_init_2, method="trf", args=args_2, bounds=([-np.inf, -np.inf, -np.inf, -np.inf, 0], [np.inf, np.inf, np.inf, np.inf, Tmax]))

        # print(solution)
        a = solution.x
        vT, xT = self.get_tocorba_vel_pos(pI_np, vI_np, a, Umax, a[4])
        print(xT, vT, a[4])

        plan_2 = []
        for t in range(0, 41, 1):
            values = self.get_tocorba_vel_pos(pI_np, vI_np, a, Umax, a[4]/40*t)
            plan_2.append(values)
            # print(values[1])
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

        return #req_vel_x, req_vel_y, req_vel_w
    

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


    def calc_1DOF_optimal_times(self, xI, xF, vI, vF, Umax):
        print("CALC 1_DOF")
        type = 1
        tt = -(2*vI - math.sqrt(2)*math.sqrt(vF**2 + vI**2 + 2*Umax*xF - 2*Umax*xI))/(2*Umax)
        T = (vI - vF + 2*Umax*tt)/Umax

        if tt > T or not np.isreal(tt):
            type = 2
            tt = (2*vI + math.sqrt(2)*math.sqrt(vF**2 + vI**2 - 2*Umax*xF + 2*Umax*xI))/(2*Umax)
            T = (vF - vI + 2*Umax*tt)/Umax
        
        return T, tt
    

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
        s24 = math.log((s25*s7 + t*s1 + s10 + kEpsilonSq) / (s25*s26 + s10 + kEpsilonSq))
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

while True:
    print(".")
    tracker_data = structure(s_tracker.recv_json(), tm.TrackerWrapperPacket)
    time = tracker_data.tracked_frame.timestamp
    # print(tracker_data.tracked_frame.timestamp)
    print(tracker_data.tracked_frame.frame_number)
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
