import math
from scipy.optimize import root, least_squares
import numpy as np
from numpy import linalg

import time as time
timer = time.time()

tsocs_params = {"B_MIN": float,
                "K1": float,
                "K2": float,
                "T_K_reg": float}

tsocs_a: list = [1, 1, 1, 1]
tsocs_T: float = 1

xIg = np.array([0, 0])
vIg = np.array([0, 0])
xFg = np.array([0, 0])
vFg = np.array([0, 0])
Umax = 1

def update(pI_np, pF_np, vI_np, vF_np, acc):
    global xIg, vIg, xFg, vFg, Umax
    xIg = pI_np
    xFg = pF_np
    vIg = vI_np
    vFg = vF_np
    Umax = acc


    global timer
    dT = time.time() - timer
    timer = time.time()

    global tsocs_a, tsocs_T
    tsocs_a[2] += tsocs_a[0]*dT
    tsocs_a[3] += tsocs_a[1]*dT
    tsocs_T -= dT


    # plan_1 = []
    # for t in range(0, 21, 1):
    #     values = get_vel_pos(pI_np, vI_np, tsocs_a, Umax, tsocs_T/20*t)
    #     plan_1.append(values)
    # draw_tocorba_speeds = {
    #     "tocorba plan 1 speeds": {
    #         "data": [
    #             {
    #                 "type": "line",
    #                 "x_list": [p[1][0]*1000, (p[1][0]+p[0][0])*1000],
    #                 "y_list": [-p[1][1]*1000, -(p[1][1]+p[0][1])*1000],
    #                 "color": "#00FFFF",
    #                 "width": 5,
    #             } for p in plan_1],
    #         "is_visible": True,
    #     }
    # }
    # s_draw.send_json(draw_tocorba_speeds)
    # draw_tocorba_plan = {
    #     "tocorba plan 1": {
    #         "data": [
    #             {
    #                 "type": "line",
    #                 "x_list": [p[1][0]*1000 for p in plan_1],
    #                 "y_list": [-p[1][1]*1000 for p in plan_1],
    #                 "color": "#0000FF",
    #                 "width": 10,
    #             },
    #         ],
    #         "is_visible": True,
    #     }
    # }
    # s_draw.send_json(draw_tocorba_plan)


    success_2, a_2, T = tsocs_stage_2(tsocs_a, pI_np, pF_np, vI_np, vF_np, Umax, tsocs_T)
    if not success_2:
        success_1, a_0, a_1, Tmax = tsocs_stage_1(pI_np, pF_np, vI_np, vF_np, Umax)
        success_2, a_2, T = tsocs_stage_2(a_1, pI_np, pF_np, vI_np, vF_np, Umax, Tmax)


        # plan_0 = []
        # for t in range(0, 21, 1):
        #     values = get_vel_pos(pI_np, vI_np, a_0, Umax, Tmax/20*t)
        #     plan_0.append(values[1])
        # draw_tocorba_plan = {
        #     "tocorba plan 0": {
        #         "data": [
        #             {
        #                 "type": "line",
        #                 "x_list": [p[0]*1000 for p in plan_0],
        #                 "y_list": [-p[1]*1000 for p in plan_0],
        #                 "color": "#FFFFFF",
        #                 "width": 10,
        #             },
        #         ],
        #         "is_visible": True,
        #     }
        # }
        # s_draw.send_json(draw_tocorba_plan)

        if success_2:
            tsocs_a = a_2
            tsocs_T = T
    else:
        tsocs_a = a_2
        tsocs_T = T


def tsocs_stage_1(pI, pF, vI, vF, Umax):
    theta = math.atan2(pF[1] - pI[1], pF[0] - pI[0])
    T, ttX, signX = calc_1DOF_optimal_times_2(pI[0], pF[0], vI[0], vF[0], Umax)
    T, ttY, signY = calc_1DOF_optimal_times_2(pI[1], pF[1], vI[1], vF[1], Umax)
    a_0 = [1, -2, 3, -4]
    a_0[2] = math.cos(theta)#*signX
    a_0[3] = math.sin(theta)#*signY
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
        calc_cost_1, 
        a_0, 
        method="lm", 
        max_nfev=50, 
        ftol=1e-6, 
        xtol=1e-6, 
        gtol=1e-6, 
        args=args,
        )
    return solution.success, a_0, solution.x, Tmax

def tsocs_stage_2(a_1, pI, pF, vI, vF, Umax, Tmax):
    # time.sleep(0)
    a_init = [a_1[0], a_1[1], a_1[2], a_1[3], Tmax]
    args = [pI, pF, vI, vF, Umax, Tmax]
    solution = least_squares(
        calc_cost_2, 
        a_init, 
        method="lm", 
        max_nfev=20, 
        ftol=1e-15, 
        xtol=1e-15, 
        gtol=1e-15, 
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

def calc_cost_2(a, xI, xF, vI, vF, Umax, T_e):
    # time.sleep(0)
    # print("CALC COST_2")
    cost = calc_cost_1(a, xI, xF, vI, vF, Umax, a[4])

    B = max(1 - linalg.norm(vF - vI)/(Umax*T_e), tsocs_params["B_MIN"])

    reg = tsocs_params["K1"] * math.exp(tsocs_params["K2"]*(a[4]/(T_e + 1e20) - tsocs_params["T_K_reg"]))

    return [cost[0], cost[1], B*cost[2], B*cost[3], reg]


def calc_cost_1(a, xI, xF, vI, vF, Umax, T):
    # time.sleep(0)
    # print("CALC COST_1")
    vT, xT = get_vel_pos(T, xI, vI, a, Umax)
    return [
        xF[0] - xT[0],
        xF[1] - xT[1],
        vF[0] - vT[0],
        vF[1] - vT[1],
        ]
    

def calc_1DOF_optimal_times_2(xI, xF, vI, vF, Umax):
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

def get_vel_pos_t(t):
    return get_vel_pos(t, xIg, vIg, tsocs_a, Umax)

def get_vel_pos(t, xI, vI, a, Umax):
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
    kEpsilonSq = 1e-30
    s30 = (s25*s7 + t*s1 + s10 + kEpsilonSq) / (s25*s26 + s10 + kEpsilonSq)
    if s30 <= 0:
        s24 = -1e30
    else:
        s24 = math.log(s30)
    s27 = s25*s10
    s28 = s25*s25*s25
    s29 = s28*s25*s25
    
    vtx, xtx = calc_tocorba_vel_pos(xI[0], vI[0], Umax, t, a1, a2, a3, a4, s1, s7, s10, s24, s25, s26, s27, s28, s29)
    vty, xty = calc_tocorba_vel_pos(xI[1], vI[1], Umax, t, a2, a1, a4, a3, s1, s7, s10, s24, s25, s26, s27, s28, s29)

    return np.array([vtx, vty]), np.array([xtx, xty])


def calc_tocorba_vel_pos(xI, vI, Umax, t, a1, a2, a3, a4, s1, s7, s10, s24, s25, s26, s27, s28, s29):
    s6 = a2*a3 - a1*a4
    s21 = a2*s6

    v = vI + Umax*( a1*(s7 - s26)/s1 + s21*s24/(s1*s25) )

    x = xI + vI*t + Umax*( a1*(s7*(s27 + t*s28) + s6*s6*s24 \
    - s26*(s27 + 2*t*s28))/(2*s29) \
    + s21*(s24*(s1*t + s10) - s25*(s7 - s26))/s29 )

    return v, x