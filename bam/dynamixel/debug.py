from .dynamixel import DynamixelActuatorV1
import time

port = "/dev/ttyUSB2"
id = 1
kp = 32

dxl = DynamixelActuatorV1(port, id)

goal_position = -2
torque_enable = True

while True:
    goal_position +=0.1
    dxl.set_goal_position(goal_position)
    dxl.set_torque(torque_enable)
    dxl.set_p_gain(kp)

    #res = dxl.packetHandler.ping(dxl.portHandler, dxl.id)
    #print("ping ->", res)  # bei Erfolg: Modellnummer, sonst result/error

    try:
        data = dxl.read_data()
        print(f"Data: {data}")
    except: 
        print("Error")

    time.sleep(0.1)