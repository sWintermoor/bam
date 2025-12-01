import os
import json
from dotenv import load_dotenv
from scipy import interpolate
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand

load_dotenv()

if __name__ == "__main__":
    uri = os.getenv("GEFILTERTER_BAG")

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri, "mcap"), rosbag2_py.ConverterOptions("", ""))

    joints = ["RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch", "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll", "LAnkleRoll"]

    jointStates = []
    jointCommands = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            dic = {
                "RHipYaw": msg.position[8],
                "LHipYaw": msg.position[9],
                "RHipRoll": msg.position[10],
                "LHipRoll": msg.position[11],
                "RHipPitch": msg.position[12],
                "LHipPitch": msg.position[13],
                "RKnee": msg.position[14],
                "LKnee": msg.position[15],
                "RAnklePitch": msg.position[16],
                "LAnklePitch": msg.position[17],
                "RAnkleRoll": msg.position[18],
                "LAnkleRoll": msg.position[19],
                "timestamp": t
            }
            jointStates.append(dic)
        if topic == "/DynamixelController/command":
            msg = deserialize_message(data, JointCommand)
            #print(msg)
            try:
                dic = {'LHipYaw': msg.positions[0], 
                    'LHipRoll': msg.positions[1], 
                    'LHipPitch': msg.positions[2], 
                    'LKnee': msg.positions[3], 
                    'LAnklePitch': msg.positions[4], 
                    'LAnkleRoll': msg.positions[5],
                        'RHipYaw': msg.positions[6],
                        'RHipRoll': msg.positions[7], 
                        'RHipPitch': msg.positions[8], 
                        'RKnee': msg.positions[9], 
                        'RAnklePitch': msg.positions[10], 
                        'RAnkleRoll': msg.positions[11],
                        'timestamp': t}
                jointCommands.append(dic)
            except IndexError:
                pass
        #print(f"Topic: {topic}, Timestamp: {t}, Data length: {len(data)}, Msg: {msg}")

    print(len(jointStates))
    print(len(jointCommands))

    print(f"First Timestamp JointStates: {jointStates[0]['timestamp']}")
    print(f"First Timestamp JointCommands: {jointCommands[0]['timestamp']}")
    print(f"Last Timestamp JointStates: {jointStates[-1]['timestamp']}")
    print(f"Last Timestamp JointCommands: {jointCommands[-1]['timestamp']}")

    print(f"Timestemp Range JointStates: {jointStates[-1]['timestamp'] - jointStates[0]['timestamp']}")
    print(f"Timestemp Range JointCommands: {jointCommands[-1]['timestamp'] - jointCommands[0]['timestamp']}")

    new_timestamps = np.arange(jointCommands[0]['timestamp'], jointCommands[-1]['timestamp'], 1000000) 

    timestamps_js = [entry['timestamp'] for entry in jointStates]
    timestamps_jc = [entry['timestamp'] for entry in jointCommands]

    jointStatesInterpolated = {}
    jointCommandsInterpolated = {}

    #print(f"JointStates: {jointStates}")

    for joint in joints:
        for topic in "jointStates", "jointCommands":
            if topic == "jointStates":
                x = timestamps_js
                y = [entry[joint] for entry in jointStates] 
                interpolated_store = jointStatesInterpolated
            else:
                x = timestamps_jc
                y = [entry[joint] for entry in jointCommands] 
                interpolated_store = jointCommandsInterpolated

            interpolated_store[joint] = interpolate.interp1d(
                x,
                y,
                kind="previous", # Use previous value if no value exists
                bounds_error=False,
                fill_value=np.nan,  # Fill values ​​that are outside the start and end timestamp with NaN
            )(new_timestamps)

    entries = []

    for index, timestamp in enumerate(new_timestamps):
        entry = {"timestamp": int(timestamp)}
        for joint in joints:
            entry[joint] = {
                "position": float(jointStatesInterpolated[joint][index]),
                "goal_position": float(jointCommandsInterpolated[joint][index]),
            }
        entries.append(entry)

    entries = entries[len(entries)//5: (len(entries)//5) + 1000]  

    daten = {
        "kp": 32, 
        "entries": entries
    }

    with open("./Wolfgang/log.json", "w") as f:
        json.dump(daten, f, indent=4)

    #print(jointCommands)

