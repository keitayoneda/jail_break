from bmx055 import BMX055
import time

def calibrateAccXYZ(bmx, n=100):
    x = []
    y = []
    z = []
    for i in range(n):
        bmx.updateAcc()
        acc = bmx.getAcc()
        x.append(acc[0])
        y.append(acc[1])
        z.append(acc[2])
        time.sleep(0.01)
    x_offset = sum(x)/len(x)
    y_offset = sum(y)/len(y)
    z_offset = sum(z)/len(z)
    print(f"acc_offset (x, y, z) : {x_offset} , {y_offset} , {z_offset}")
    return x_offset, y_offset, z_offset
    
def calibrateGyroXYZ(bmx, n=100):
    x = []
    y = []
    z = []
    for i in range(n):
        bmx.updateGyro()
        gyro = bmx.getGyro()
        x.append(gyro[0])
        y.append(gyro[1])
        z.append(gyro[2])
        time.sleep(0.01)
    x_offset = sum(x)/len(x)
    y_offset = sum(y)/len(y)
    z_offset = sum(z)/len(z)
    print(f"gyro_offset (x, y, z) : {x_offset} , {y_offset} , {z_offset}")
    return x_offset, y_offset, z_offset

def calibrateMagnetXYZ(bmx, n=100):
    x = []
    y = []
    z = []
    for i in range(n):
        bmx.updateMagnet()
        magnet = bmx.getMagnet()
        x.append(magnet[0])
        y.append(magnet[1])
        z.append(magnet[2])
        time.sleep(0.01)
    x_offset = sum(x)/len(x)
    y_offset = sum(y)/len(y)
    z_offset = sum(z)/len(z)
    print(f"magnet_offset (x, y, z) : {x_offset} , {y_offset} , {z_offset}")
    return x_offset, y_offset, z_offset
 
def main():
    bmx = BMX055()
    calibrateAccXYZ(bmx)

if __name__ == "__main__":
    main()

