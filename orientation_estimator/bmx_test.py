import smbus
import time
import copy

ACC_I2C_ADDRESS = 0x19
ACC_REGISTER_ADDRESS = 0x02

def getByte(i2c, i2c_address, register_address):
    lsb = i2c.read_byte_data(i2c_address, register_address)
    msb = i2c.read_byte_data(i2c_address, register_address+1)
    is_updated = lsb & 0x01
    return lsb, msb, is_updated
    

def cvtByte2Value(lsb, msb):
    value = (msb*16) + ((lsb & 0xF0) / 16)
    if value < 2048:
        return value
    else:
        return value - 4096

def getAcc(i2c):
    x_lsb, x_msb, x_is_updated = getByte(i2c, ACC_I2C_ADDRESS, ACC_REGISTER_ADDRESS)
    x_value = cvtByte2Value(x_lsb, x_msb)
    y_lsb, y_msb, y_is_updated = getByte(i2c, ACC_I2C_ADDRESS, ACC_REGISTER_ADDRESS+2)
    y_value = cvtByte2Value(y_lsb, y_msb)
    z_lsb, z_msb, z_is_updated = getByte(i2c, ACC_I2C_ADDRESS, ACC_REGISTER_ADDRESS+4)
    z_value = cvtByte2Value(z_lsb, z_msb)
    
    factor = 9.8 *1e-3

    x_acc = x_value*factor
    y_acc = y_value*factor
    z_acc = z_value*factor
    return x_acc, y_acc, z_acc

class BMX055:
    def __init__(self):
        self.i2c = smbus.SMBus(1)
        self.acc_i2c_address = 0x19
        self.acc_register_address = 0x02
        self.gyro_i2c_address = 0x69
        self.gyro_register_address = 0x02
        self.magnet_i2c_address = 0x13
        self.magnet_register_address = 0x02
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.magnet = [0.0, 0.0, 0.0]

    def __del__(self):
        self.i2c.close()

    def _readByte(self, i2c_address, lsb_address):
        lsb = self.i2c.read_byte_data(i2c_address, lsb_address)
        msb = self.i2c.read_byte_data(i2c_address, lsb_address+1)
        is_updated = lsb & 0x01
        return lsb, msb

    def _cvt12Byte2Value(self, lsb, msb):
        value = (msb*2**4) + ((lsb & 0xF0) / 16)
        if value < 2**11:
            return value
        else:
            return value - 2**12

    def _calcAcc(self, value):
        factor = 9.8 *1e-3
        return value*factor

    def _cvt16Byte2Value(self, lsb, msb):
        value = msb*(2**8) + lsb 
        if value < 2**15:
            return value
        else:
            return value - 2**16
        return value

    def _calcGyro(self, value):
        factor = 2000/(2**15)
        return value*factor

    
    def updateAcc(self):
        for i in range(3):
            lsb, msb = self._readByte(self.acc_i2c_address, self.acc_register_address + 2*i)
            value = self._cvt12Byte2Value(lsb, msb)
            self.acc[i] = self._calcAcc(value)

    def updateGyro(self):
        for i in range(3):
            lsb, msb = self._readByte(self.gyro_i2c_address, self.gyro_register_address+2*i)
            value = self._cvt16Byte2Value(lsb, msb)
            self.gyro[i] = self._calcGyro(value)


    def getAcc(self):
        return copy.copy(self.acc)
    
    def printAcc(self):
        print(f"acc: (x,y,z)=({self.acc[0]:1.3f}, {self.acc[1]:1.3f}, {self.acc[2]:1.3f})")

    def getGyro(self):
        return copy.copy(self.gyro)

    def printGyro(self):
        print(f"gyro: (x,y,z)=({self.gyro[0]:1.3f}, {self.gyro[1]:1.3f}, {self.gyro[2]:1.3f})")


def main():
    bmx055 = BMX055()
    integrated_val = [0, 0, 0]
    dt = 0.01
    print_dt = 0.2
    prev_print_time = time.time()
    while True:
        bmx055.updateAcc()
        bmx055.updateGyro()
        time.sleep(dt)
        # bmx055.printGyro()
        gyro = bmx055.getGyro()
        integrated_val[0] += gyro[0]*dt
        integrated_val[1] += gyro[1]*dt
        integrated_val[2] += gyro[2]*dt
        if time.time() - prev_print_time >= print_dt:
            prev_print_time = time.time()
            print(integrated_val)


if __name__ == "__main__":
    main()
