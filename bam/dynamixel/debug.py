from .dynamixel import DynamixelActuatorV1
import time

port = "/dev/ttyUSB2"
id = 1
kp = 32

dxl = DynamixelActuatorV1(port, id)

goal_position = 0
torque_enable = True

cnt = 1

while True:
    print(f"Data: {dxl.read_data()}")
    #except:
     #   pass
    #time.sleep(0.1)

    #print(f"Modulo Operation: {cnt % 10}")

    if cnt % 10 == 0:
        goal_position +=0.1
        dxl.set_goal_position(goal_position)
        dxl.set_torque(torque_enable)
        dxl.set_p_gain(kp)
    cnt += 1

"""
while True:
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

    goal_position -= 0.01
    time.sleep(0.1)
"""