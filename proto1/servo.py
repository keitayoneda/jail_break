from dynamixel_driver import dynamixel_io
import sys
import time
import math
from typing import List

def checkConnection(dxl_io:dynamixel_io.DynamixelIO, ids:List[int]):
    for each_id in ids:
        if dxl_io.ping(each_id):
            print('[SUCCESS] : id %d respond to a ping' % each_id)
        else:
            print('[ERROR] : id %d not respond.' % each_id)
            sys.exit(1)

class SingleDxlServo:
    def __init__(self, dxl_io:dynamixel_io.DynamixelIO, dxl_id:int):
        self.io:dynamixel_io.DynamixelIO = dxl_io
        self.id:int = dxl_id
        self.min_angle:float = -150
        self.max_angle:float = 150
        self.vel_scale = 0.111 #val->rot/min
    def __calcAngleFromVal(self, value:int):
        # 0-1024までの値を取るvalueをmin_angleからmax_angleまでの値に変換する
        ret_angle = value/1024 * (self.max_angle - self.min_angle) + self.min_angle
        return ret_angle
    def __calcValFromAngle(self, angle:float):
        if angle > self.max_angle:
            angle = self.max_angle
        elif angle < self.min_angle:
            angle = self.min_angle
        # 0-1024までの値を取るvalueをmin_angleからmax_angleまでの値に変換する
        ret_val = (angle-self.min_angle)/ (self.max_angle - self.min_angle) * 1024 
        return int(ret_val)

    def __calcValFromOmega(self, omega:float):
        val = omega/(2*math.pi/60)/self.vel_scale
        val = int(val)
        if val > 1023:
            val = 1023
        elif val < 0:
            val = 0
        return val

    def __calcOmegaFromVal(self, val:int):
        omega = val*self.vel_scale*(360/60)
        return omega

    def setAngle(self, dest_angle:float):
        dest_val = self.__calcValFromAngle(dest_angle)
        self.res = self.io.set_position(self.id, dest_val)

    def setAngleAndOmega(self, dest_angle:float, dest_omega:float):
        dest_angle_val = self.__calcValFromAngle(dest_angle)
        dest_omega_val = self.__calcValFromOmega(dest_omega)
        self.res = self.io.set_position_and_speed(self.id, dest_angle_val, dest_omega_val)


    def getAngle(self):
        ret_val = self.io.get_position(self.id)
        return self.__calcAngleFromVal(ret_val)

    def getOmega(self):
        ret_val = self.io.get_speed(self.id)
        return self.__calcOmegaFromVal(ret_val)

def test():
    port = "/dev/ttyUSB0"
    baudrate = 1000000
    dxl_id = 1
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR:', soe)
        sys.exit(1)

    ids = [dxl_id]
    checkConnection(dxl_io, ids) 

    servo = SingleDxlServo(dxl_io, dxl_id)

    for _ in range(3):
        start = time.time()
        while time.time() - start < 6:
            t = time.time() - start
            dest_angle = 90*math.sin(t*math.pi)
            dest_omega = 90*math.cos(t*math.pi)
            servo.setAngleAndOmega(dest_angle, dest_omega)
            print("angle=", servo.getAngle())
            print("omega=", servo.getOmega())
            time.sleep(0.001)

if __name__ == "__main__":
    test()
