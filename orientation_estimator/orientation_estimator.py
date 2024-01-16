from bmx055 import BMX055
from ekf import EKF
from data_server import DataServer
from bmx_calib import calibrateAccXYZ, calibrateGyroXYZ, calibrateMagnetXYZ
import time
import numpy as np

# quaternionはx, y, z, wの順
def getScewMat(wx, wy, wz):
    return np.array([[0, -wz, wy, wx],
                     [wz, 0, -wx, wy],
                     [-wy, wx, 0, wz],
                     [-wx, -wy, -wz, 0]])

def rotate(quat, v):
    qv = quat[:3]
    qw = quat[3,0]
    rot_v = (qw**2 - np.linalg.norm(qv)**2)*v + 2*(qv.T @ v)*qv + 2*qw*np.cross(qv, v, axis=0)
    return rot_v
    
def getAFunc(bmx, dt):
    def AFunc():
        gyro = bmx.getGyro()
        #gyro[2] = 0
        Ac = 0.5 * getScewMat(*gyro)
        return np.eye(4) + Ac * dt

    return AFunc

def getCFunc(g, mag):
    def CFunc(q):
        qx = q[0, 0]
        qy = q[1, 0]
        qz = q[2, 0]
        qw = q[3, 0]
        gx = g[0]
        gy = g[1]
        gz = g[2]
        magx = mag[0]
        magy = mag[1]
        magz = mag[2]

        C = np.zeros((6, 4))
        C[0, 0] = 2*qx*gx + 2*qy*gy + 2*qz*gz
        C[0, 1] = 2*qw*gz + 2*qx*gy - 2*qy*gx
        C[0, 2] = -2*qw*gy + 2*qx*gz - 2*qz*gx
        C[0, 3] = 2*qw*gx + 2*qy*gz - 2*qz*gy

        C[1, 0] = -2*qw*gz - 2*qx*gy + 2*qy*gx
        C[1, 1] = 2*qx*gx + 2*qy*gy + 2*qz*gz
        C[1, 2] = 2*qw*gx + 2*qy*gz - 2*qz*gy
        C[1, 3] = 2*qw*gy - 2*qx*gz + 2*qz*gx

        C[2, 0] = 2*qw*gy - 2*qx*gz + 2*qz*gx
        C[2, 1] = -2*qw*gx - 2*qy*gz + 2*qz*gy
        C[2, 2] = 2*qx*gx + 2*qy*gy + 2*qz*gz
        C[2, 3] = 2*qw*gz + 2*qx*gy - 2*qy*gx

        C[3, 0] = 2*qx*magx + 2*qy*magy + 2*qz*magz
        C[3, 1] = 2*qw*magz + 2*qx*magy - 2*qy*magx
        C[3, 2] = -2*qw*magy + 2*qx*magz - 2*qz*magx
        C[3, 3] = 2*qw*magx + 2*qy*magz - 2*qz*magy

        C[4, 0] = -2*qw*magz - 2*qx*magy + 2*qy*magx
        C[4, 1] = 2*qx*magx + 2*qy*magy + 2*qz*magz
        C[4, 2] = 2*qw*magx + 2*qy*magz - 2*qz*magy
        C[4, 3] = 2*qw*magy - 2*qx*magz + 2*qz*magx

        C[5, 0] = 2*qw*magy - 2*qx*magz + 2*qz*magx
        C[5, 1] = -2*qw*magx - 2*qy*magz + 2*qz*magy
        C[5, 2] = 2*qx*magx + 2*qy*magy + 2*qz*magz
        C[5, 3] = 2*qw*magz + 2*qx*magy - 2*qy*magx
        return C
    return CFunc

def getObsFunc(bmx):
    def obsFunc():
        acc = np.array(bmx.getAcc())[:, np.newaxis].reshape(3,1)
        acc = acc / np.linalg.norm(acc) * 9.8
        obs_g = acc
        # print(f"obs_g : {obs_g}")
        mag = np.array(bmx.getMagnet())[:, np.newaxis].reshape(3,1)
        mag = mag / np.linalg.norm(mag) *9.8
        obs_mag = mag

        obs_vec = np.zeros((6,1))
        obs_vec[:3] = obs_g
        obs_vec[3:6] = obs_mag

        return obs_vec
    return obsFunc

def getPredFunc(g, mag):
    g_vec = np.array(g)[:, np.newaxis].reshape(3,1)
    mag_vec = np.array(mag)[:,np.newaxis].reshape(3,1)
    mag_vec = mag_vec / np.linalg.norm(mag_vec)
    def predFunc(q):
        rot_g = rotate(q, g_vec)
        rot_mag = rotate(q, mag_vec)
        # print(f"rot_g : {rot_g}")
        rot_vec = np.zeros((6,1))
        print("rot_g", rot_g)
        rot_vec[:3] = rot_g
        rot_vec[3:6] = rot_mag
        return rot_vec
    return predFunc


def calcNTheta(quat):
    s = np.linalg.norm(quat[:3])
    c = quat[3, 0]
    theta = np.arctan2(s, c)*2
    if theta > np.pi:
        theta -= np.pi*2
    n_vec = quat[:3] / np.sin(theta*0.5)
    return n_vec, theta



def main():
    host = "192.168.12.50"
    port = 20021
    data_server = DataServer(host, port)
    data_server.startAndDetach()

    np.set_printoptions(suppress=True)
    bmx = BMX055()
    x_offset, y_offset, z_offset = calibrateAccXYZ(bmx)
    bmx.acc_offset=[x_offset, y_offset, z_offset - 9.8]
    x_offset, y_offset, z_offset = calibrateGyroXYZ(bmx)
    #bmx.gyro_offset=[x_offset, y_offset, z_offset]
    x_offset, y_offset, z_offset = calibrateMagnetXYZ(bmx)
    g_base = np.array([0, 0, 9.8])
    mag_base = np.array([x_offset, y_offset, z_offset])
    mag_base = mag_base / np.linalg.norm(mag_base) * 9.8

    dt = 0.1
    print_interval = 0.1
    A_func = getAFunc(bmx, dt)
    C_func = getCFunc(g_base, mag_base)
    pred_func = getPredFunc(g_base, mag_base)
    obs_func = getObsFunc(bmx)
    initial_x = np.array([[0.0], [0.0], [0.0], [1.0]])
    initial_cov = np.diag([0.5, 0.5, 0.5, 0.5])
    update_cov = np.diag([0.05, 0.05, 0.05, 0.05])
    obs_cov = np.diag([0.001, 0.001, 0.001, 0.1, 0.1, 0.1])
    ekf = EKF(initial_x, initial_cov, A_func, C_func, pred_func, update_cov, obs_cov)
    
    prev_time = time.time()
    prev_print_time = time.time()
    while True:
        prev_time = time.time()
        bmx.updateAcc()
        bmx.updateGyro()
        bmx.updateMagnet()
        ekf.update()
        ekf.normalizeX()
        ekf.observe(obs_func())
        ekf.normalizeX()
        est_q = ekf.x
        mag_x, mag_y, mag_z = bmx.getMagnet()
        abs_mag = (mag_x**2 + mag_y**2 + mag_z**2)**0.5
        q_str = f"{est_q[3, 0]} {est_q[0, 0]} {est_q[1,0]} {est_q[2, 0]} {mag_x/abs_mag} {mag_y/abs_mag} {mag_z/abs_mag}" # w, x, y, z の順に並べる
        data_server.setSendData(q_str)
        cycle_time = time.time() - prev_time
        sleep_time = dt - cycle_time
        if sleep_time < 0:
            print(f"cycle time exceed dt={dt}, cycle_time={cycle_time}")
        else:
            time.sleep(sleep_time)



if __name__ == "__main__":
    main()

