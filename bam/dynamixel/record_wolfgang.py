import json
import datetime
import os
import numpy as np
import argparse
import time
from .dynamixel import DynamixelActuatorV1
from .trajectory_Wolfgang import *

NUMBER_DYNAMIXELS = 20

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="quintic_walk")
arg_parser.add_argument("--kp", type=int, default=32)
arg_parser.add_argument("--speed", type=float, default=1.0)
args = arg_parser.parse_args()

dxl_dic = {}
for index in range(NUMBER_DYNAMIXELS):
    dxl_dic[index + 1] = DynamixelActuatorV1(args.port, id=index + 1)
    dxl_dic[index + 1].set_p_gain(args.kp)
    dxl_dic[index + 1].set_torque(True)

trajectory = trajectories[args.trajectory]

try:
    for index in range(NUMBER_DYNAMIXELS):
        dxl_dic[index + 1].read_data()
except:
    for index in range(NUMBER_DYNAMIXELS):
        dxl_dic[index + 1].set_torque(False)
    exit()

start = time.time()
goal_dic = {}
while time.time() - start < 1.0:
    for index in range(NUMBER_DYNAMIXELS):
        goal_dic[index + 1] = trajectory(0)
        dxl_dic[index + 1].set_goal_position(goal_dic[index + 1])

start = time.time()
data = {
    "kp": args.kp,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": []
}

while (time.time() - start) * args.speed < trajectory.init_duration + trajectory.traj_duration:
    t = (time.time() - start) * args.speed

    for index in range(NUMBER_DYNAMIXELS):
        goal_dic[index + 1] = trajectory(t)
        dxl_dic[index + 1].set_goal_position(goal_dic[index + 1])
    time.sleep(0.001)

    if t >= trajectory.init_duration:
        t0 = time.time() - start
        for index in range(NUMBER_DYNAMIXELS):
            entry = {f"dxl_{index + 1}": dxl_dic[index + 1].read_data()}
        t1 = time.time() - start

        entry["timestamp"] = (t0 + t1) / 2.0
        for index in range(NUMBER_DYNAMIXELS):
            entry[f"dxl_{index + 1}"]["goal_position"] = goal_dic[index + 1]
        data["entries"].append(entry)

for index in range(NUMBER_DYNAMIXELS):
    dxl_dic[index + 1].set_torque(False)

# Format YYYY-MM-DD_HH:mm:ss
date = datetime.datetime.now().strftime("%d_%Hh%Mm%S")

filename = f"{args.logdir}/{args.trajectory}_{args.kp}_{args.motor}_{date}.json"
json.dump(data, open(filename, "w"))
