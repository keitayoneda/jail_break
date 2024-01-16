import servo
import dynamixel_driver.dynamixel_io as dynamixel_io
import sys
import time

class Climber:
    def __init__(self, servos):
        self.servos = servos
        self.signs = [-1, 1, -1, 1]
        self.omega = 2
        self.upper_angle = 0
        self.lower_angle = 25
        self.max_angle = 30

    def leg(self, angle, index):
        current_angle = self.servos[index].getAngle()
        sign = self.signs[index]
        if (angle < current_angle):
            sign *= -1
        self.servos[index].setAngleAndOmega(self.signs[index]*angle, self.omega)
        
    def init_pose(self):
        for i in range(4):
            self.leg(self.max_angle, i)
        time.sleep(1.0)

    def climb(self):
        for i in range(4):
            self.leg(-self.lower_angle, i)
        time.sleep(1.0)

    def elevate_leg(self):
        for i in range(2):
            self.leg(self.max_angle, i)
            self.leg(self.max_angle, i+2)
            time.sleep(1.0)
            self.leg(self.upper_angle, i)
            self.leg(self.upper_angle, i+2)
            time.sleep(1.0)

def main():
    port = "/dev/ttyUSB0"
    baudrate = 1000000
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR', soe)
        sys.exit(1)

    ids = [1,2,3,4]
    servo.checkConnection(dxl_io, ids)
    # servo_names = ["lf", "lb", "rb", "rf"]
    servos = [servo.SingleDxlServo(dxl_io, dxl_id) for dxl_id in ids]
    # servo_dict = {}
    # for key, value in zip(servo_names, servos):
    #     servo_dict[key] = value

    climber = Climber(servos)
    #climber.init_pose()
    time.sleep(1)
    climber.climb()
    climber.elevate_leg()



if __name__ == "__main__":
    main()


