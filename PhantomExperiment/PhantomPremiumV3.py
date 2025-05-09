#!/usr/bin/env python

import argparse
import mujoco as mj
from mujoco import viewer
import numpy as np
import os
import time
import math
from copy import deepcopy
from scipy.io import savemat
from scipy.io import loadmat
import csv
from scipy import optimize
import matplotlib.pyplot as plt
import time
import pickle

# ---- Global and default variables ----------
simend = 20.0
e = np.zeros(3)
e_dot = np.zeros(3)
C_final = np.array([-np.pi / 2, 0, 0])  # data.qpos gives the radian position
home_pos = np.array([0, 0, 0])
i = 0
M = np.zeros((4, 4))

q1 = []
q2 = []
q3 = []


def forward_kinematics(q):
    l2 = 0.208
    l3 = 0.168

    # Calculate forward kinematics of each component
    x = np.sin(q[:, 0]) * (l2 * np.cos(q[:, 1]) + l3 * np.sin(q[:, 2]))
    y = l3 - l3 * np.cos(q[:, 2]) + l2 * np.sin(q[:, 1])
    z = -l2 + np.cos(q[:, 0]) * (l2 * np.cos(q[:, 1]) + l3 * np.sin(q[:, 2]))

    # Combine results into a single output array
    X = np.column_stack((x, y, z))
    return X


def trajectory(time, t_final, C_final, home_pos):
    """Generates trajectory position, velocity, and acceleration."""
    a2 = 3 / t_final**2
    a3 = -2 / t_final**3
    Cposition = (a2 * time**2 + a3 * time**3) * C_final
    Cvelocity = (2 * a2 * time + 3 * time**2 * a3) * C_final
    Cacceleration = (2 * a2 + 6 * time * a3) * C_final

    return Cposition, Cvelocity, Cacceleration


def lineTracking(model, data):
    # Desired trajectory for position, velocity, and acceleration
    qDes, qdDes, qddDes = trajectory(data.time, simend, C_final, home_pos)

    # Current state of the system
    q = data.qpos
    qd = data.qvel
    # Control gains
    Kp = 100  # proportional gain
    Kd = 1  # derivative gain

    # Error calculation
    e = qDes - q
    edot = qdDes - qd
    # Compute desired torque

    # Compute the computed torque
    tau = Kd * edot + Kp * e
    print(qDes[0])
    # Apply computed torque to the actuated joints
    data.ctrl = tau


# def lineTracking(model, data):
#     Kp = 10
#     Kd = 1
#     qDes, qdDes, qddDes = trajectory(data.time,simend,C_final, home_pos)
#     e = qDes - np.array([data.qpos[0], data.qpos[1],data.qpos[3] ])
#     edot = qdDes - np.array([data.qvel[0], data.qvel[1],data.qvel[3] ])
#     tauD = Kp * e + Kd * edot
#     print(data.qpos)

#     # Send the control output to the system
#     data.qfrc_applied[3] =  1


def joint_space_impedance_controller(model, data):
    global i
    JG1 = 0.0042
    JG2 = 0.00641
    JG3 = 0.00347
    t = data.time

    # Send the control output to the system
    # data.qfrc_applied[0] =
    data.qfrc_applied += np.array([q1[i, 0] * JG1, q2[i, 0] * JG2, 0, q3[i, 0] * JG3])

    i = i + 1
    if i >= len(q1):
        i = 10000  # Reset to the beginning of the data


# called each time around the loop
def simandcollect(dum):
    mj.mj_step(model, data)


if __name__ == "__main__":
    # Path to the robot XML file
    xml_path = os.path.join(os.path.dirname(__file__), "PhantomPremiumV3.xml")

    # Initialise MuJoCo data structures
    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)

    parser = argparse.ArgumentParser(
        description="Load a mujoco file, run a simulation and save data for matlab."
    )

    parser.add_argument(
        "-v", "--viewer", action="store_true", help="use the mjuoco viewer"
    )  # viewer

    parser.add_argument(
        "-x",
        "--mjmodel",
        type=str,
        default=xml_path,
        help="Will use named file as the mjmodel.If not given uses a default file",
    )  # model file

    parser.add_argument(
        "-r", "--runtime", type=float, default=simend, help="set the runtime in seconds"
    )  # runtime

    parser.add_argument(
        "-l", "--log", action="store_true", help="log the data into csv files"
    )  # csv log

    parser.add_argument(
        "-m", "--dotmat", action="store_true", help="log the data into a matlab matrix"
    )  # matlab logs may be faster

    parser.add_argument(
        "-s",
        "--numsensors",
        type=int,
        default=-1,
        help="t.b.d. save the sensor data. This number must be less than or equal to the number of sensors.",
    )  # t.b.d.

    parser.add_argument(
        "-c",
        "--controller",
        type=int,
        choices=[1, 2, 3, 4],
        required=False,
        help="Choose a controller option: 1 - Joint Space Impedance Controller, 2 - Cartesian Space Impedance Controller, 3 - Losey's Impedance Controller, 4 - Iterative Learning Controller",
    )  # controller selection

    args = parser.parse_args()

    # ---- Main program starts here -----------------------------------------
# Site id to get sensor information about end-effector

# Set Initial joint positions
# data.qpos = q_init


# set the controller
def set_controller():
    controllers = {
        1: joint_space_impedance_controller,
        2: lineTracking,
    }

    if args.controller is not None:
        mj.set_mjcb_control(controllers[args.controller])

    else:
        try:
            print("Choose a controller option:")
            print("1 - Joint Space Impedance Controller")
            print("2 - Line Tracking")

            choice = int(input("Enter the number of your choice: "))

            if choice in controllers:
                selected_controller = controllers[choice]
                mj.set_mjcb_control(selected_controller)
                print(f"Controller {choice} has been set successfully.")
            else:
                print("Invalid choice. Please enter a number between 1 and 4.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")


# Example usage
set_controller()

if args.viewer:  # viewer is the -v option
    with mj.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat = [-0.3, 1, 0.1]
        viewer.cam.elevation = -5
        viewer.user_scn.ngeom = 0
        viewer.cam.azimuth = 110

        start = time.time()
        # viewer.cam.elevation = -20
        while viewer.is_running() and (time.time() - start) < args.runtime:  #
            simandcollect(0)

            viewer.sync()
            elapsedtime = time.time() - start
            if (
                data.time > elapsedtime
            ):  # comment out next two lines to not attempt real time
                time.sleep(data.time - elapsedtime)
        print(f"model time=%0.2f, actual time=%0.2f" % (data.time, time.time() - start))
#      time_until_next_step = model.opt.timestep - (time.time() - step_start)
else:  # no viewer
    while data.time < args.runtime:
        simandcollect(0)
