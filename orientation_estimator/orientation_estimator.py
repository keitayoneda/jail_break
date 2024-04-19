from bmx055 import BMX055
from ekf import EKF
from data_server import DataServer
from bmx_calib import calibrateAccXYZ, calibrateGyroXYZ, calibrateMagnetXYZ
import time
import numpy as np

# quaternionはx, y, z, wの順


def getScewMat(wx, wy, wz):
    wz *= -1
    return np.array([[0, -wx, wy, -wz],
                     [wx, 0, -wz, wy],
                     [wy, wz, 0, -wx],
                     [wz, -wy, wx, 0]])


def rotate(quat, v):
    q_w = quat[0, 0]
    q_x = quat[1, 0]
    q_y = quat[2, 0]
    q_z = quat[3, 0]
    v_x = v[0]
    v_y = v[1]
    v_z = v[2]
    rot_v = np.zeros((3, 1))
    rot_v[0] = q_w*(q_w*v_x + q_y*v_z - q_z*v_y) + q_x*(q_x*v_x + q_y*v_y + q_z*v_z) + \
        q_y*(q_w*v_z + q_x*v_y - q_y*v_x) - q_z*(q_w*v_y - q_x*v_z + q_z*v_x)
    rot_v[1] = q_w*(q_w*v_y - q_x*v_z + q_z*v_x) - q_x*(q_w*v_z + q_x*v_y - q_y*v_x) + \
        q_y*(q_x*v_x + q_y*v_y + q_z*v_z) + q_z*(q_w*v_x + q_y*v_z - q_z*v_y)
    rot_v[2] = q_w*(q_w*v_z + q_x*v_y - q_y*v_x) + q_x*(q_w*v_y - q_x*v_z + q_z*v_x) - \
        q_y*(q_w*v_x + q_y*v_z - q_z*v_y) + q_z*(q_x*v_x + q_y*v_y + q_z*v_z)
    return rot_v


def rotateQuat(p, q):
    ret_quat = np.zeros((4,1))

    p_w = p[0, 0]
    p_x = p[1, 0]
    p_y = p[2, 0]
    p_z = p[3, 0]
    
    q_w = q[0, 0]
    q_x = q[1, 0]
    q_y = q[2, 0]
    q_z = q[3, 0]

    ret_quat[0, 0] = p_w*q_w - p_x*q_x - p_y*q_y - p_z*q_z
    ret_quat[1, 0] = p_w*q_x + p_x*q_w + p_y*q_z - p_z*q_y
    ret_quat[2, 0] = p_w*q_y - p_x*q_z + p_y*q_w + p_z*q_x
    ret_quat[3, 0] = p_w*q_z + p_x*q_y - p_y*q_x + p_z*q_w

    return ret_quat
    
def dettiageRotate(quat):
    # X軸のZ軸回り回転量が0になるようにでっち上げる
    x_vec = np.array([1.0, 0.0, 0.0])
    rot_x_vec = rotate(quat, x_vec).reshape(3,)
    th = np.arctan2(rot_x_vec[1], rot_x_vec[0])
    # Z軸回りに-th回転するquaternionをquatにかける
    delta_quat = np.array([[np.cos(-th/2)], [0.0], [0.0], [np.sin(-th/2)]])
    ret_quat = rotateQuat(delta_quat, quat)
    return ret_quat


def getAFunc(bmx, dt):
    def AFunc():
        gyro = bmx.getGyro()
        # gyro[2] = 0
        Ac = 0.5 * getScewMat(*gyro)
        return np.eye(4) + Ac * dt
    return AFunc


def getCFunc(v):
    def CFunc(q):
        q_w = q[0, 0]
        q_x = q[1, 0]
        q_y = q[2, 0]
        q_z = q[3, 0]
        v_x = v[0]
        v_y = v[1]
        v_z = v[2]

        C = np.zeros((3, 4))
        C[0, 0] = 2*q_w*v_x + 2*q_y*v_z - 2*q_z*v_y
        C[0, 1] = 2*q_x*v_x + 2*q_y*v_y + 2*q_z*v_z
        C[0, 2] = 2*q_w*v_z + 2*q_x*v_y - 2*q_y*v_x
        C[0, 3] = -2*q_w*v_y + 2*q_x*v_z - 2*q_z*v_x
        C[1, 0] = 2*q_w*v_y - 2*q_x*v_z + 2*q_z*v_x
        C[1, 1] = -2*q_w*v_z - 2*q_x*v_y + 2*q_y*v_x
        C[1, 2] = 2*q_x*v_x + 2*q_y*v_y + 2*q_z*v_z
        C[1, 3] = 2*q_w*v_x + 2*q_y*v_z - 2*q_z*v_y
        C[2, 0] = 2*q_w*v_z + 2*q_x*v_y - 2*q_y*v_x
        C[2, 1] = 2*q_w*v_y - 2*q_x*v_z + 2*q_z*v_x
        C[2, 2] = -2*q_w*v_x - 2*q_y*v_z + 2*q_z*v_y
        C[2, 3] = 2*q_x*v_x + 2*q_y*v_y + 2*q_z*v_z
        return C
    return CFunc


def getObsFunc(bmx):
    def obsFunc():
        acc = np.array(bmx.getAcc())[:, np.newaxis].reshape(3, 1)
        acc = acc / np.linalg.norm(acc) * 9.8
        obs_g = acc

        obs_vec = np.zeros((3, 1))
        obs_vec[:3] = obs_g
        return obs_vec
    return obsFunc


def getPredFunc(g):
    g_vec = np.array(g)[:, np.newaxis].reshape(3, 1)

    def predFunc(q):
        rot_g = rotate(q, g_vec)
        rot_vec = np.zeros((3, 1))
        rot_vec[:3] = rot_g
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
    data_server = DataServer()
    data_server.startAndDetach()

    np.set_printoptions(suppress=True)
    bmx = BMX055()
    x_offset, y_offset, z_offset = calibrateAccXYZ(bmx)
    bmx.acc_offset = [x_offset, y_offset, z_offset - 9.8]
    x_offset, y_offset, z_offset = calibrateGyroXYZ(bmx)
    bmx.gyro_offset=[x_offset, y_offset, z_offset]
    x_offset, y_offset, z_offset = calibrateMagnetXYZ(bmx)
    g_base = np.array([0, 0, 9.8])
    mag_base = np.array([x_offset, y_offset, z_offset])
    mag_base = mag_base / np.linalg.norm(mag_base) * 9.8

    dt = 0.1
    print_interval = 0.1
    A_func = getAFunc(bmx, dt)
    C_func = getCFunc(g_base)
    pred_func = getPredFunc(g_base)
    obs_func = getObsFunc(bmx)
    initial_x = np.array([[1.0], [0.0], [0.0], [0.0]])
    initial_cov = np.diag([0.5, 0.5, 0.5, 0.5])
    update_cov = np.diag([0.05, 0.05, 0.05, 0.05])
    obs_cov = np.diag([0.001, 0.001, 0.001])
    ekf = EKF(initial_x, initial_cov, A_func,
              C_func, pred_func, update_cov, obs_cov)

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
        # est_q = ekf.x
        est_q = dettiageRotate(ekf.x)
        rot_g = rotate(est_q, [0, 0, 9.8])
        # print(f"rot_x = {rot_g.reshape(3,)}")
        rot_x = rotate(est_q, [100, 0, 0])
        # print(f"rot_x = {rot_x.reshape(3,)}")
        rot_y = rotate(est_q, [0, 100, 0])
        # print(f"rot_y = {rot_y.reshape(3,)}")
        rot_z = rotate(est_q, [0, 0, 100])
        # print(f"rot_z = {rot_z.reshape(3,)}")
        mag_x, mag_y, mag_z = bmx.getMagnet()
        abs_mag = (mag_x**2 + mag_y**2 + mag_z**2)**0.5
        # w, x, y, z の順に並べる
        q_str = f"{est_q[0, 0]} {est_q[1,0]} {est_q[2, 0]} {est_q[3, 0]} {mag_x/abs_mag} {mag_y/abs_mag} {mag_z/abs_mag}"
        data_server.setSendData(q_str)
        cycle_time = time.time() - prev_time
        sleep_time = dt - cycle_time
        if sleep_time < 0:
            print(f"cycle time exceed dt={dt}, cycle_time={cycle_time}")
        else:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
