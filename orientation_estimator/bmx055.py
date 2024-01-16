import smbus
import time
import copy

class BMX055:
    def __init__(self):
        self.i2c = smbus.SMBus(1)
        self.acc_i2c_address = 0x19
        self.acc_register_address = 0x02
        self.gyro_i2c_address = 0x69
        self.gyro_register_address = 0x02
        self.magnet_i2c_address = 0x13
        self.magnet_register_address = 0x42
        # acc
        # -2g~2g
        self.i2c.write_byte_data(self.acc_i2c_address, 0x0f, 0x03)
        # 125Hz
        self.i2c.write_byte_data(self.acc_i2c_address, 0x10, 0x0c)

        # magnet
        self.i2c.write_byte_data(self.magnet_i2c_address, 0x4b, 0x01) # power bit turn to 1
        self.i2c.write_byte_data(self.magnet_i2c_address, 0x4c, 0x00) # OpMode 0x00
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.magnet = [0.0, 0.0, 0.0]

        self.acc_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]

    def __del__(self):
        self.i2c.close()

    def _readByte(self, i2c_address, lsb_address):
        lsb = self.i2c.read_byte_data(i2c_address, lsb_address)
        msb = self.i2c.read_byte_data(i2c_address, lsb_address+1)
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
        factor = 2000/(2**15)*3.141592/180
        return value*factor

    
    def updateAcc(self):
        for i in range(3):
            lsb, msb = self._readByte(self.acc_i2c_address, self.acc_register_address + 2*i)
            value = self._cvt12Byte2Value(lsb, msb)
            self.acc[i] = self._calcAcc(value)-self.acc_offset[i]


    def updateGyro(self):
        for i in range(3):
            lsb, msb = self._readByte(self.gyro_i2c_address, self.gyro_register_address+2*i)
            value = self._cvt16Byte2Value(lsb, msb)
            self.gyro[i] = self._calcGyro(value) - self.gyro_offset[i]

    def updateMagnet(self):
        value = [0, 0, 0]
        lsb_x, msb_x = self._readByte(self.magnet_i2c_address, self.magnet_register_address) 
        value[0] = (msb_x*(2**8)+(lsb_x & 0xF8))/8
        lsb_y, msb_y = self._readByte(self.magnet_i2c_address, self.magnet_register_address+2) 
        value[1] = (msb_y*(2**8)+(lsb_y & 0xF8))/8
        lsb_z, msb_z = self._readByte(self.magnet_i2c_address, self.magnet_register_address+4) 
        value[2] = (msb_z*(2**8)+(lsb_z & 0xFE))/2
            
        if value[0] > 2**12:
            value[0] -= 2**13
        if value[1] > 2**12:
            value[1] -= 2**13
        if value[2] > 2**14:
            value[2] -= 2**15

        for i in range(3):
            self.magnet[i] = value[i] * 16*1e-6

    def getAcc(self):
        return copy.copy(self.acc)
    
    def printAcc(self):
        print(f"acc: (x,y,z)=({self.acc[0]:1.3f}, {self.acc[1]:1.3f}, {self.acc[2]:1.3f})")

    def getGyro(self):
        return copy.copy(self.gyro)

    def printGyro(self):
        print(f"gyro: (x,y,z)=({self.gyro[0]:1.3f}, {self.gyro[1]:1.3f}, {self.gyro[2]:1.3f})")


    def getMagnet(self):
        return copy.copy(self.magnet)

    def printMagnet(self):
        print(f"magnet: (x,y,z)=({self.magnet[0]}, {self.magnet[1]}, {self.magnet[2]})")

    def printMagnetScale(self):
        abs_mag = (self.magnet[0]**2 + self.magnet[1]**2 + self.magnet[2]**2)**0.5
        print(f"magnet: (x,y,z)=({self.magnet[0]/abs_mag}, {self.magnet[1]/abs_mag}, {self.magnet[2]/abs_mag})")

def main():
    bmx055 = BMX055()
    integrated_val = [0, 0, 0]
    dt = 0.1
    while True:
        bmx055.updateAcc()
        bmx055.updateGyro()
        bmx055.updateMagnet()
        bmx055.printMagnetScale()
        time.sleep(dt)
        acc  = bmx055.getAcc()
        abs_acc = (acc[0]**2 + acc[1]**2 + acc[2]**2)**0.5
        bmx055.printAcc()
        # bmx055.printGyro()
        gyro = bmx055.getGyro()
        integrated_val[0] += gyro[0]*dt
        integrated_val[1] += gyro[1]*dt
        integrated_val[2] += gyro[2]*dt
        #print(integrated_val)


if __name__ == "__main__":
    main()
