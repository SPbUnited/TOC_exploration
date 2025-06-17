import zmq
from attrs import define
from cattrs import structure, unstructure

import robot_command_model as rcm
import tracker_model as tm

import math
import numpy as np

import socket

import time

import csv

import threading
import sys

import tsocs
import bangbang

context = zmq.Context()
s_tracker = context.socket(zmq.SUB)
s_tracker.connect("ipc:///tmp/transnet.tracker")
s_tracker.setsockopt_string(zmq.SUBSCRIBE, "")

s_control = context.socket(zmq.PUB)
s_control.connect("ipc:///tmp/ether.signals.xsub")

s_draw = context.socket(zmq.PUB)
s_draw.connect("ipc:///tmp/ether.draw.xsub")
tsocs.s_draw = s_draw

s_telemetry = context.socket(zmq.PUB)
s_telemetry.connect("ipc:///tmp/ether.telemetry.xsub")

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# UDP_IP = "10.0.120.211"
# UDP_PORT = 10000

def create_packet(
    bot_number: int,  # unsigned byte (0-255)
    coord_x: int,  # signed byte (-128 to 127)
    coord_y: int,  # signed byte
    angle: int,  # хз
    beep: int,  # signed byte
) -> bytes:
    # Convert all values to bytes and pack into a
    coord_x_abs = abs(coord_x)
    coord_y_abs = abs(coord_y)
    bytes_list = [
        0x01,  # Header
        bot_number,
        coord_x_abs.to_bytes(1, "big")[0], #speed x
        coord_y_abs.to_bytes(1, "big")[0], #speed y
        angle.to_bytes(1, "big", signed=True)[0], #speed r
        0, # dribbler speed
        0, #kicker voltage
        0, # kick up
        0,  #kick down
        beep, # beep
        int(coord_x < 0), # drib enable
        int(coord_y < 0), # kick enable
        0, # autokick
    ]

    return bytes(bytes_list)

elapsed1 = []
elapsed2 = []

# points = [np.array([-1.5, -0.12]), np.array([-1, 0.12]), np.array([-0.5, -0.12]), np.array([0, 0.12]), np.array([1, 0]), np.array([0, 1]), np.array([-3, 2])]
# velocities = [np.array([0, 0]), np.array([0.2, 0]), np.array([0.2, 0]), np.array([0.2, 0]), np.array([0.3, 0.4]), np.array([0.3, 0.4]), np.array([-0.5, -0.2])]
start_vel_x = 0
start_vel_y = 0.5
points = [np.array([0, 0]), np.array([1, 0])]
velocities = [np.array([0, 0.5]), np.array([0.5, 0])]
current_start = 0
current_goal = 1

EPS = 0.05
alpha = 0.7
COORD_EPS = 0.05
SPEED_EPS = 0.5

trajectory: list[np.array] = []
trajectory_max_len = 50
trajectory_update_int = 1
update_counter: int = 0

MAX_ACC = 0.4
MAX_VEL = 1

tsocs.tsocs_params = {"B_MIN": 0.05,
                      "K1": 50,
                      "K2": 0.1,
                      "T_K_reg": 1.4}

BOT_NUMBER = 0

PLANER_FREQ     = 0.1
CONTROLLER_FREQ = 50

BANGBANG_PLANNER    = 0
TSOCS_PLANNER       = 1
planner_type = TSOCS_PLANNER

preTrjUpdateTimer = time.time()
trjUpdateTimer = time.time()

FF_Kp = 1

TIME_K = 1000/1000

dT = 1/CONTROLLER_FREQ
dT_K = 1

x = 0
y = 0
vel_x = 0
vel_y = 0

trjX = 0
trjY = 0

def update_trajectory():
    print("start")
    plannerTime = time.time()

    global trjUpdateTimer
    trjUpdateTimer = preTrjUpdateTimer

    pos = np.array([x, y])
    vel = np.array([vel_x, vel_y])

    if current_goal == current_start:
        pos = points[0]
        vel = np.array([start_vel_x, start_vel_y])

    time0 = time.time()
    tsocs.update(pos, points[current_goal], vel, velocities[current_goal], MAX_ACC)
    time1 = time.time()
    
    print("TSOCS " + str((time1-time0)*1000))
    elapsed1.append((time1-time0)*1000)
    if len(elapsed1) > 1000: 
        elapsed1.pop(0)


    # max_speed = 0
    plan_2 = []
    with open('tsocs.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for t in range(0, 101, 1):
            values = tsocs.get_vel_pos_t(tsocs.tsocs_T/100*t)
            plan_2.append(values)
            csvwriter.writerow([values[1][0], values[1][1]])
    #     if linalg.norm(values[0]) > max_speed:
    #         max_speed = linalg.norm(values[0])

    # draw_tocorba_speeds = {
    #     "tocorba plan 2 speeds": {
    #         "data": [
    #             {
    #                 "type": "line",
    #                 "x_list": [p[1][0]*1000, (p[1][0]+p[0][0])*1000],
    #                 "y_list": [-p[1][1]*1000, -(p[1][1]+p[0][1])*1000],
    #                 "color": "#FFFF00",
    #                 "width": 5,
    #             } for p in plan_2],
    #         "is_visible": True,
    #     }
    # }
    # s_draw.send_json(draw_tocorba_speeds)
    draw_tocorba_plan = {
        "tocorba plan 2": {
            "data": [
                {
                    "type": "line",
                    "x_list": [p[1][0]*1000 for p in plan_2],
                    "y_list": [-p[1][1]*1000 for p in plan_2],
                    "color": "#FFAA00",
                    "width": 30,
                },
            ],
            "is_visible": True,
        }
    }
    s_draw.send_json(draw_tocorba_plan)
    

    time0 = time.time()
    bangbang.update(pos, points[current_goal], vel, velocities[current_goal], MAX_ACC, MAX_VEL)
    time1 = time.time()
    print("bangbang " + str((time1-time0)*1000))
    elapsed2.append((time1-time0)*1000)
    if len(elapsed2) > 1000: 
        elapsed2.pop(0)
    
    draw_speeed_data = {
        "speeeed": {
            "data": [
                {
                    "type": "arrow",
                    "x": float(0),
                    "y": float(0),
                    "dx": float(vel[0]*1000),
                    "dy": float(-vel[1]*1000),
                    "color": "#0000FF",
                    "width": 5,
                },
            ],
            "is_visible": True,
        }
    }
    s_draw.send_json(draw_speeed_data)

    plan = []
    with open('bang_s.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for t in range(0, 101, 1):
            values = bangbang.get_bang_bang_values(bangbang.T/100*t)
            plan.append(values[2])
            csvwriter.writerow([values[2][0], values[2][1]])
    draw_planned_data = {
            "plan": {
                "data": [
                    {
                        "type": "line",
                        "x_list": [float(p[0]*1000) for p in plan],
                        "y_list": [float(-p[1]*1000) for p in plan],
                        "color": "#FF00FF",
                        "width": 10,
                    },
                ],
                "is_visible": True,
            }
        }
    s_draw.send_json(draw_planned_data)

    plannerTime = time.time() - plannerTime
    data = {"Computation time": 
            " TSOCS avg time:     {0:.3f}ms (min: {1:.3f}ms, max: {2:.3f}ms)\n".format(np.average(elapsed1), min(elapsed1), max(elapsed1)) +
            " Bang-bang avg time: {0:.3f}ms (min: {1:.3f}ms, max: {2:.3f}ms)\n".format(np.average(elapsed2), min(elapsed2), max(elapsed2)) +
            " Planner time: {0:.3f}ms\n".format(plannerTime*1000)
            }
    s_telemetry.send_json(data)
    
    print("end")
    # while True:
    #     print("-")


def update_vision():
    tracker_data = structure(s_tracker.recv_json(), tm.TrackerWrapperPacket)
    while True:
        try:
            new_tracker_data = structure(s_tracker.recv_json(flags=zmq.NOBLOCK), tm.TrackerWrapperPacket)
            tracker_data = new_tracker_data
        except:
            break

    tracked_robot = None

    for r in tracker_data.tracked_frame.robots:
        if r.robot_id.team == tm.Team.YELLOW and r.robot_id.id == BOT_NUMBER:
            tracked_robot = r

    if tracked_robot is None:
        return
    
    global x, y, angle, vel_x, vel_y, vel_angle
    x = tracked_robot.pos.x
    y = tracked_robot.pos.y
    angle = tracked_robot.orientation
    vel_x = tracked_robot.vel.x/TIME_K
    vel_y = tracked_robot.vel.y/TIME_K
    vel_angle = tracked_robot.vel_angular

    draw_speeed_data = {
        "speeeed2": {
            "data": [
                {
                    "type": "arrow",
                    "x": float(0),
                    "y": float(0),
                    "dx": float(vel_x*1000),
                    "dy": float(-vel_y*1000),
                    "color": "#FF0000",
                    "width": 5,
                },
            ],
            "is_visible": True,
        }
    }
    s_draw.send_json(draw_speeed_data)

    global preTrjUpdateTimer
    preTrjUpdateTimer = time.time()

    pos = np.array([x, y])
    vel = np.array([vel_x, vel_y])
    print(np.linalg.norm(vel))

    global current_goal, current_start, update_counter

    err = alpha*np.linalg.norm(pos - points[current_goal]) + (1 - alpha)*np.linalg.norm(vel - velocities[current_goal]) 

    if current_goal == current_start:
        err = alpha*np.linalg.norm(pos - points[0]) + (1 - alpha)*np.linalg.norm(vel - velocities[0]) 
    # print(err)
    if err < EPS:
        if current_goal == current_start:
            current_start = 0
            current_goal = 1
        else:
            current_start = current_goal
            current_goal = (current_goal + 1)%len(points)
            update_trajectory()
            planner_timer = time.time()

    update_counter += 1
    if update_counter > trajectory_update_int:
        update_counter = 0
        trajectory.append(pos)
        if len(trajectory) > trajectory_max_len:
            trajectory.pop(0)

    draw_points_data = {
        "points": {
            "data": [ 
                {
                    "type": "circle",
                    "x": float(p[0]*1000),
                    "y": float(-p[1]*1000),
                    "radius": 50,
                    "color": "#0000FF" if (p == points[current_goal]).all() else "#FF0000",
                } for p in points],
            "is_visible": True,
        }
    }
    draw_speeds_data = {
        "speeds": {
            "data": [
                {
                    "type": "arrow",
                    "x": float(z[0][0]*1000),
                    "y": float(-z[0][1]*1000),
                    "dx": float(z[1][0]*1000),
                    "dy": float(-z[1][1]*1000),
                    "color": "#0000FF" if (z[0] == points[current_goal]).all() else "#FF0000",
                    "width": 20,
                }  for z in zip(points, velocities)],
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
                        "x_list": [float(p[0]*1000) for p in trajectory],
                        "y_list": [float(-p[1]*1000) for p in trajectory],
                        "color": "#888888",
                        "width": 10,
                    },
                ],
                "is_visible": True,
            }
        }
    s_draw.send_json(draw_trajectory_data)


def update_control():

    t = (time.time() - trjUpdateTimer) + dT*dT_K
    if planner_type == BANGBANG_PLANNER:
        acc, vel, coord = bangbang.get_bang_bang_values(t)
    elif planner_type == TSOCS_PLANNER:
        vel, coord = tsocs.get_vel_pos_t(t)

    global trjX, trjY
    trjX = coord[0]
    trjY = coord[1]

    if current_goal == current_start:
        req_vel_x = start_vel_x*math.cos(angle) + start_vel_y*math.sin(angle)
        req_vel_y = -start_vel_x*math.sin(angle) + start_vel_y*math.cos(angle)
    else:
        req_vel_x = vel[0] + (coord[0] - x)*FF_Kp
        req_vel_y = vel[1] + (coord[1] - y)*FF_Kp
        req_vel_x = req_vel_x*math.cos(angle) + req_vel_y*math.sin(angle)
        req_vel_y = -req_vel_x*math.sin(angle) + req_vel_y*math.cos(angle)
    req_vel_w = -angle*5

    control_data = rcm.RobotControlExt(
        isteamyellow=True,
        robot_commands=[
            rcm.RobotCommand(
                id=0,
                move_command=rcm.RobotMoveCommand(
                    local_velocity=rcm.MoveLocalVelocity(
                        forward=float(req_vel_x/TIME_K),
                        left=float(req_vel_y/TIME_K),
                        # forward=float(1),
                        # left=float(0),
                        angular=float(req_vel_w),
                    ),
                ),
                kick_speed=0,
                kick_angle=0,
                dribbler_speed=0,
            ),
        ],
    )
    s_control.send_json({"transnet": "actuate_robot", "data": unstructure(control_data)})

    # if abs(tracked_robot.pos.x) > 4000/2 or abs(tracked_robot.pos.y) > 2500/2:
    #     print("Плохие корды, ниче не отправляю")
    #     continue

    # packet = create_packet(
    #     BOT_NUMBER,
    #     int(req_vel_x*100), # int от 0 до 255
    #     int(req_vel_y*100), # int от 0 до 255
    #     0, # int от -127 до 127
    #     1,
    # )
    # print("отправил", packet)
    # sock.sendto(packet, (UDP_IP, UDP_PORT))


def planner():
    while True:
        planner_timer = time.time()
        
        print("planer: ")
        try:
            update_trajectory()
        except:
            print("tyajelo")

        time.sleep(max(0, 1/PLANER_FREQ  - (time.time() - planner_timer)))


def controller():
    with open('path.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        program_start_time = time.time()

        while True:
            controller_timer = time.time()

            try:
                print("controller: ")
                update_vision()
                # update_trajectory()
                update_control()
                csvwriter.writerow([time.time() - program_start_time, x, y, trjX, trjY])
            except:
                print("tyajelo")

            time.sleep(max(0, 1/CONTROLLER_FREQ - (time.time() - controller_timer)))

sys.setswitchinterval(0.005)
threads = []
threads.append(threading.Thread(target=planner))
threads.append(threading.Thread(target=controller))

for t in threads:
    t.start()
