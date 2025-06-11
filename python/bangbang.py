import math
from scipy.optimize import root, least_squares
import numpy as np
from numpy import linalg

import time

v_max = np.array([0, 0])
T = 0

xIg = np.array([0, 0])
vIg = np.array([0, 0])
xFg = np.array([0, 0])
vFg = np.array([0, 0])
max_acc = 1
max_vel = 1

def update(pos, goal, vel, vel_goal, acc, sp):
    global v_max, T
    v_max = max_vel_bang_bang(pos, goal, vel, vel_goal)
    T = get_bang_bang_time(pos, goal, v_max, vel, vel_goal)
    print("Bang-bang time = " + str(T))

    global xIg, vIg, xFg, vFg, max_acc, max_vel
    xIg = pos
    xFg = goal
    vIg = vel
    vFg = vel_goal
    max_acc = acc
    max_vel = sp


def short_dist(v_max: np.ndarray, args: list) -> np.ndarray:
    # time.sleep(0)
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


def long_dist(ang: np.ndarray, args: list) -> np.ndarray:
    # time.sleep(0)
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
    r_start: np.array,
    r_end: np.array,
    v_start: np.array = np.array([0, 0]),
    v_end: np.array = np.array([0, 0]),
    ) -> np.array:
    """Найти макс. скорость бенг-бенга, однозначно его задает"""

    max_speed = max_vel
    a_max = max_acc

    delta_r = np.subtract(r_end, r_start)
    v_max_initial = np.array([0, 0])
    if np.linalg.norm(delta_r) != 0:
        v_max_initial = np.array([delta_r[0], delta_r[1]]) / np.linalg.norm(delta_r) * max_speed
    args = [np.array([v_start[0], v_start[1]]), np.array([v_end[0], v_end[1]]), np.array([delta_r[0], delta_r[1]]), a_max]
    short_solution = root(short_dist, v_max_initial, args=args)

    if not short_solution.success:
        short_solution = least_squares(short_dist, v_max_initial, method="lm", args=[args])

    v_max = np.array([short_solution.x[0], short_solution.x[1]])

    if np.linalg.norm(v_max) > max_speed:
        args.insert(0, max_speed)
        ang_initial = np.array([math.atan2(delta_r[1], delta_r[0])])
        long_solution = root(long_dist, ang_initial, args=args)
        if not long_solution.success:
            long_solution = least_squares(long_dist, ang_initial, method="lm", args=[args])
        angle = long_solution.x[0]
        v_max = np.array([max_speed * np.cos(angle), max_speed * np.sin(angle)])

    return v_max


def get_bang_bang_time(
    r_start: np.array,
    r_end: np.array,
    v_max: np.array,
    v_start: np.array = np.array([0, 0]),
    v_end: np.array = np.array([0, 0]),
    ) -> float:
    """Найти время проезда по бенг-бенгу"""

    max_speed = max_vel
    a_max = max_acc

    return (
        np.linalg.norm(v_max - v_start) / a_max
        + np.linalg.norm(
            r_end
            - r_start
            - ((v_max + v_start) * np.linalg.norm(v_max - v_start) + (v_end + v_max) * np.linalg.norm(v_end - v_max)) / 2 / a_max
        )
        / max_speed
        + np.linalg.norm(v_end - v_max) / a_max
    )

def get_bang_bang_values(
    t: float,
) -> list[np.array]:
    """Возвращает кинематические величины в любой момент проезда по бенг-бенгу"""
    r_start = xIg
    r_end = xFg
    v_start = vIg
    v_end = vFg
    max_speed = max_vel
    a_max = max_acc

    p1 = r_start + (v_max + v_start) * np.linalg.norm(v_max - v_start) / 2 / a_max
    p2 = r_end - (v_end + v_max) * np.linalg.norm(v_end - v_max) / 2 / a_max
    t1 = np.linalg.norm(v_max - v_start) / a_max
    t2 = t1 + np.linalg.norm(p2 - p1) / max_speed
    t3 = t2 + np.linalg.norm(v_end - v_max) / a_max
    values: list[np.array] = []
    if t < 0:
        values.append((v_max - v_start)/np.linalg.norm(v_max - v_start) * a_max)
        values.append(v_start)
        values.append(r_start)
    if t < t1:
        values.append((v_max - v_start)/np.linalg.norm(v_max - v_start) * a_max)
        values.append(v_start + values[0] * t)
        values.append(r_start + v_start * t + values[0] * t**2 / 2)
    elif t < t2:
        values.append(np.array([0, 0]))
        values.append(v_max)
        values.append(p1 + v_max * (t - t1))
    elif t < t3:
        values.append((v_end - v_max)/np.linalg.norm(v_end - v_max) * a_max)
        values.append(v_max + values[0] * (t - t2))
        values.append(p2 + v_max * (t - t2) + values[0] * (t - t2) ** 2 / 2)
    else:
        values.append((v_end - v_max)/np.linalg.norm(v_end - v_max) * a_max)
        values.append(v_end)
        values.append(r_end)
    return values