import zmq
from attrs import define
from cattrs import structure, unstructure

import robot_command_model as rcm
import tracker_model as tm

import math

context = zmq.Context()
s_tracker = context.socket(zmq.SUB)
s_tracker.connect("ipc:///tmp/transnet.tracker")
s_tracker.setsockopt_string(zmq.SUBSCRIBE, "")

s_control = context.socket(zmq.PUB)
s_control.connect("ipc:///tmp/ether.signals.xsub")

print("Control decoder example")

last_frame_number = 0


@define
class Robot:
    target_x: float
    target_y: float
    target_angle: float

    kp: float
    kpa: float

    def update(self, x, y, angle):
        req_glob_vel_x = self.kp * (self.target_x - x)
        req_glob_vel_y = self.kp * (self.target_y - y)
        req_vel_x = math.cos(angle) * req_glob_vel_y - math.sin(angle) * req_glob_vel_x
        req_vel_y = math.sin(angle) * req_glob_vel_y + math.cos(angle) * req_glob_vel_x

        req_vel_w = (self.target_angle - angle) % (2 * math.pi) * self.kpa
        return req_vel_x, req_vel_y, req_vel_w


robot = Robot(target_x=3, target_y=-2, target_angle=2, kp=1, kpa=1)

while True:
    print(".")
    tracker_data = structure(s_tracker.recv_json(), tm.TrackerWrapperPacket)
    print(tracker_data.tracked_frame.frame_number)
    print(tracker_data.tracked_frame.frame_number - last_frame_number)
    print(tracker_data.tracked_frame.robots[0])

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
                    # global_velocity=rcm.MoveGlobalVelocity(
                    #     x=1,
                    #     y=0,
                    #     angular=0,
                    # ),
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
