import socket
import time
import pyglet
import numpy as np
import quaternion
import threading


def rotate(quat, v):
    q_w = quat[0]
    q_x = quat[1]
    q_y = quat[2]
    q_z = quat[3]
    v_x = v[0]
    v_y = v[1]
    v_z = v[2]
    rot_v = [0 for _ in range(3)]
    rot_v[0] = q_w*(q_w*v_x + q_y*v_z - q_z*v_y) + q_x*(q_x*v_x + q_y*v_y + q_z*v_z) + \
        q_y*(q_w*v_z + q_x*v_y - q_y*v_x) - q_z*(q_w*v_y - q_x*v_z + q_z*v_x)
    rot_v[1] = q_w*(q_w*v_y - q_x*v_z + q_z*v_x) - q_x*(q_w*v_z + q_x*v_y - q_y*v_x) + \
        q_y*(q_x*v_x + q_y*v_y + q_z*v_z) + q_z*(q_w*v_x + q_y*v_z - q_z*v_y)
    rot_v[2] = q_w*(q_w*v_z + q_x*v_y - q_y*v_x) + q_x*(q_w*v_y - q_x*v_z + q_z*v_x) - \
        q_y*(q_w*v_x + q_y*v_z - q_z*v_y) + q_z*(q_x*v_x + q_y*v_y + q_z*v_z)
    return rot_v


window = pyglet.window.Window(400, 400)
batch = pyglet.graphics.Batch()

x_vec = np.array([100, 0, 0])
y_vec = np.array([0, 100, 0])
z_vec = np.array([0, 0, 100])

x_axis_color = (255, 0, 0)
y_axis_color = (0, 255, 0)
z_axis_color = (0, 0, 255)
x_axis = pyglet.shapes.Line(
    0, 0, 0, 0, width=2, color=x_axis_color, batch=batch)
y_axis = pyglet.shapes.Line(
    0, 0, 0, 0, width=2, color=y_axis_color, batch=batch)
z_axis = pyglet.shapes.Line(
    0, 0, 0, 0, width=2, color=z_axis_color, batch=batch)
mag_axis = pyglet.shapes.Line(
    0, 0, 0, 0, width=2, color=(255, 255, 255), batch=batch)

origin_x = 200
origin_y = 200


def projection(v, zplain=0):
    # 3次元ベクトルを2次元ベクトルに射影する
    return [v[0], v[1]]


def perspective(v, plain_z):
    # 3次元ベクトルを遠近法で2次元ベクトルに射影する
    z = v[2]
    alpha = plain_z / (plain_z - z)
    return [v[0] * alpha, v[1] * alpha]


def drawCoordinate(quat):
    # 3次元ベクトルを原点から描画する
    # x_vec_rotated = quaternion.rotate_vectors(quat, x_vec)
    # y_vec_rotated = quaternion.rotate_vectors(quat, y_vec)
    # z_vec_rotated = quaternion.rotate_vectors(quat, z_vec)
    x_vec_rotated = rotate(quat, x_vec)
    y_vec_rotated = rotate(quat, y_vec)
    z_vec_rotated = rotate(quat, z_vec)
    z_plain = 150
    x_vec_2d = projection(x_vec_rotated, z_plain)
    y_vec_2d = projection(y_vec_rotated, z_plain)
    z_vec_2d = projection(z_vec_rotated, z_plain)
    x_axis.x = origin_x
    x_axis.y = origin_y
    x_axis.x2 = origin_x + x_vec_2d[0]
    x_axis.y2 = origin_y + x_vec_2d[1]
    y_axis.x = origin_x
    y_axis.y = origin_y
    y_axis.x2 = origin_x + y_vec_2d[0]
    y_axis.y2 = origin_y + y_vec_2d[1]
    z_axis.x = origin_x
    z_axis.y = origin_y
    z_axis.x2 = origin_x + z_vec_2d[0]
    z_axis.y2 = origin_y + z_vec_2d[1]


def drawMagnetVec(quat, vec):
    # 3次元ベクトルを原点から描画する
    vec_rotated = rotate(quat, vec)
    z_plain = 150
    vec_2d = projection(vec_rotated, z_plain)
    mag_axis.x = origin_x
    mag_axis.y = origin_y
    mag_axis.x2 = origin_x + vec_2d[0]
    mag_axis.y2 = origin_y + vec_2d[1]


@window.event
def on_draw():
    window.clear()
    batch.draw()


def main():
    def pyglet_func():
        return pyglet.app.run()
    threading.Thread(target=pyglet_func).start()
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
    server_address = "192.168.1.211"
    server_port = 20021
    sock.connect((server_address, server_port))
    while True:
        sock.sendall(b"nyan")
        data = sock.recv(1024)
        data_str = data.decode()
        if data_str == "this is data":
            continue
        data_float_list = [float(x) for x in data_str.split()]

        if len(data_float_list) != 7:
            continue
        quat = data_float_list[:4]
        magnet_vec = data_float_list[4:]
        magnet_vec = np.array(magnet_vec) / np.linalg.norm(magnet_vec) * 100
        drawCoordinate(quat)
        drawMagnetVec(quat, magnet_vec)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
