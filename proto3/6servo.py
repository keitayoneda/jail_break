import servo
import dynamixel_driver.dynamixel_io as dynamixel_io
import sys
import time

class Climber:
    def __init__(self, servos):
        self.servos = servos
        self.signs = [1, -1, -1, 1, 1, -1]
        self.fast_omega = 6
        self.omega = 2
        self.slow_omega = 1
        self.fix_angle = [-8, -3, -8, -8, -3, -8] # 突っ張る角度
        self.lower_angle = [-27, -27, -27, -27, -27, -27] # 登るときに足を下げる角度
        self.max_angle = 42 # 足を上に戻すときに上げる角度
        self.init_angle = 42 # 最初に足をセットする角度
        self.servo_num = len(servos)

    def target_leg_fast(self, index, angle):
        self.servos[index].setAngleAndOmega(self.signs[index]*angle, self.fast_omega)

    def target_leg(self, index, angle):
        self.servos[index].setAngleAndOmega(self.signs[index]*angle, self.omega)

    def target_leg_slow(self, index, angle):
        self.servos[index].setAngleAndOmega(self.signs[index]*angle, self.slow_omega)

        
    def init_pose(self):
        for i in range(self.servo_num):
            self.target_leg(i, self.init_angle)

    def elevate_leg(self, index):
        # sideの足を上に上げる。その間それ以外の足で壁に突っ張る
        # side = 0 or 1 or 2

        for each_index in index:
            self.target_leg_fast(each_index, self.max_angle)
    
    def fix_leg(self, index):
        for each_index in index:
            self.target_leg_fast(each_index, self.fix_angle[each_index])
    def fix_leg2(self, index, angle):
        self.target_leg_fast(index, angle)

    def elevate_body(self):
        # すべての足を下に下げて胴体を上に上げる
        for each_index in range(self.servo_num):
            self.target_leg(each_index, self.lower_angle[each_index])


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
    print("elevate_body")
    climber.elevate_body()
    time.sleep(1.5)
    climber.elevate_leg([0])
    time.sleep(1.5)
    climber.fix_leg([0])
    time.sleep(1)
    climber.elevate_leg([2])
    time.sleep(1)
    climber.fix_leg([2])
    time.sleep(1)
    climber.elevate_leg([3])
    time.sleep(1)
    climber.fix_leg([3])
    time.sleep(1)
    climber.elevate_leg([5])
    time.sleep(1)
    climber.fix_leg([5])
    time.sleep(1)
    climber.elevate_leg([1, 4])
    time.sleep(1)
    climber.fix_leg([1, 4])


    print("end")

if __name__ == "__main__":
    main()


