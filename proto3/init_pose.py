import servo
import dynamixel_driver.dynamixel_io as dynamixel_io
import sys
import time

class Climber:
    def __init__(self, servos):
        self.servos = servos
        self.signs = [1, -1, -1, 1, 1, -1]
        self.omega = 2
        self.upper_angle = 0 # 突っ張る角度
        self.fix_angle = 0 # 突っ張る角度
        self.lower_angle = -25 # 登るときに足を下げる角度
        self.max_angle = 30 # 足を上に戻すときに上げる角度
        self.init_angle = 30 # 最初に足をセットする角度
        self.servo_num = len(servos)

    def target_leg(self, index, angle):
        current_angle = self.servos[index].getAngle()
        sign = self.signs[index]
        # if (angle < current_angle):
        #     sign *= -1
        self.servos[index].setAngleAndOmega(self.signs[index]*angle, self.omega)
        
    def init_pose(self):
        for i in range(self.servo_num):
            self.target_leg(i, self.init_angle)

    def elevate_leg(self, side):
        if side not in [0, 1]:
            return 
        # sideの足を上に上げる。その間それ以外の足で壁に突っ張る
        # side = 0 or 1
        index = [0, 2, 4]
        if side == 1:
            index = [1, 3, 5]

        for each_index in index:
            self.target_leg(each_index, self.max_angle)
    
    def fix_leg(self, side):
        if side not in [0, 1]:
            return
        # side = 0 or 1
        # sideの足を壁に押し当てて突っ張る
        index = [0, 2, 4]
        if side == 1:
            index = [1, 3, 5]

        for each_index in index:
            self.target_leg(each_index, self.fix_angle)

    def elevate_body(self):
        # すべての足を下に下げて胴体を上に上げる
        for each_index in range(self.servo_num):
            self.target_leg(each_index, self.lower_angle)

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
    time.sleep(1)
    print("init_pose")
    climber.init_pose()
    print("end")
    time.sleep(2)

if __name__ == "__main__":
    main()


