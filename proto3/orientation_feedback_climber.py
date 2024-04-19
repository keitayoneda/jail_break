from climber import Climber
from receive_orientation import Receiver
import servo
import dynamixel_driver.dynamixel_io as dynamixel_io
import sys
import time

def rotate(quat, v):
    q_w = quat[0]
    q_x = quat[1]
    q_y = quat[2]
    q_z = quat[3]
    v_x = v[0]
    v_y = v[1]
    v_z = v[2]
    rot_v = [0.0, 0.0, 0.0]

    rot_v[0] = q_w*(q_w*v_x + q_y*v_z - q_z*v_y) + q_x*(q_x*v_x + q_y*v_y + q_z*v_z) + q_y*(q_w*v_z + q_x*v_y - q_y*v_x) - q_z*(q_w*v_y - q_x*v_z + q_z*v_x)

    rot_v[1] = q_w*(q_w*v_y - q_x*v_z + q_z*v_x) - q_x*(q_w*v_z + q_x*v_y - q_y*v_x) + q_y*(q_x*v_x + q_y*v_y + q_z*v_z) + q_z*(q_w*v_x + q_y*v_z - q_z*v_y)
    
    rot_v[2] = q_w*(q_w*v_z + q_x*v_y - q_y*v_x) + q_x*(q_w*v_y - q_x*v_z + q_z*v_x) - q_y*(q_w*v_x + q_y*v_z - q_z*v_y) + q_z*(q_x*v_x + q_y*v_y + q_z*v_z)

    return rot_v

def calcZDelta(quat):
    pos_list = [
        [130, 160, 25],
        [0, 160, 25],
        [-130, 160, 25],
        [-130, -160, 25],
        [0, -160, 25],
        [130, -160, 25]]
    z_list = []
    for pos in pos_list:
        rot_pos = rotate(quat, pos)
        z_list.append(rot_pos[2])
    z_mean = sum(z_list)/len(z_list)
    for i in range(len(z_list)):
        z_list[i] -= z_mean
    return z_list

def calcElevateAngles(z_list):
    base_angle = -18
    feedback_gain = 0.3
    angles = []
    for i in range(len(z_list)):
        angles.append(base_angle + feedback_gain*z_list[i])
    return angles

def main():
    port = "/dev/ttyUSB0"
    baudrate = 1000000
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR', soe)
        sys.exit(1)

    ids = [1,2,3,4,5,6]
    servo.checkConnection(dxl_io, ids)
    servos = [servo.SingleDxlServo(dxl_io, dxl_id) for dxl_id in ids]

    climber = Climber(servos)
    receiver = Receiver("192.168.10.50", 20021)
    receiver.start()

    count = 0
    while True:
        if (count > 10):
            str_in = input("input action: ")
            if str_in == "q":
                break
        else:
            count += 1
        quat = receiver.quat
        z_list = calcZDelta(quat)
        print(f"z_list: {z_list}")
        angles = calcElevateAngles(z_list)
        print(f"angles: {angles}")
        # angles = [-27, -27, -27, -27, -27, -27]

        print("elevate_body")
        climber.elevate_body2(angles)
        time.sleep(0.5)
        leg_index = [[0, 3], [2, 5], [1, 4]]
        for leg in leg_index:
            climber.elevate_leg(leg)
            time.sleep(0.4)
            climber.fix_leg(leg)
            time.sleep(0.3)
    climber.init_pose()
    receiver.stop()

    print("end")

if __name__ == "__main__":
    main()
