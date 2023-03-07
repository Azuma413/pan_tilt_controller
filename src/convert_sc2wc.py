#!/usr/bin/env python
import rospy
import numpy as np
from numpy import sin, cos, tan
from geometry_msgs.msg import Point
from std_msgs.msg import float

#画像検出により目標の画像上での位置を求める(u, v)
#画像上での位置の深度情報を求める depth


#カメラでRGBイメージとDepthイメージを取得する
#RGBイメージから骨格の座標を検出する（u, v）
#Depthイメージにおける骨格の位置の値を取得する（z）
#スクリーン座標（u, v）+ zを変換し、ワールド座標系における座標（x, y, z）を得る
#入出力
#入力は以下です。

#カメラのワールド座標系における位置・回転行列 = t, R
#カメラの内部パラメータ行列 = K
#RGB画像上の（スクリーン座標系における）座標 = (u, v)
#Death画像における（=スクリーン座標系における）奥行き = z
#出力は以下です。

#実際の空間（ワールド座標系）における座標 = (x', y', z')


def convert_uvz_to_xyz(u, v, z, R, t, K):
    K_inv = np.linalg.inv(K)

    # in screen coord
    cs = np.asarray([u, v, 1])
    cs_ = cs * z

    # in camera coord
    cc = np.dot(K_inv, cs_)

    # in world coord
    cw = np.dot(R, cc) + t

    return cw

def calc_R(pitch, yaw, roll):
    a = np.radians(pitch)
    b = np.radians(yaw)
    c = np.radians(roll)

    R_x = np.asarray([
        [1, 0, 0],
        [0, cos(a), -sin(a)],
        [0, sin(a), cos(a)],
    ])

    R_y = np.asarray([
        [cos(b), 0, sin(b)],
        [0, 1, 0],
        [-sin(b), 0, cos(b)],
    ])

    R_z = np.asarray([
        [cos(c), -sin(c), 0],
        [sin(c), cos(c), 0],
        [0, 0, 1],
    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

def calc_K(fov_x, pixel_w, pixel_h, cx=None, cy=None):
    if cx is None:
        cx = pixel_w / 2.0
    if cy is None:
        cy = pixel_h / 2.0

    fx = 1.0 / (2.0 * tan(np.radians(fov_x) / 2.0)) * pixel_w
    fy = fx

    K = np.asarray([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ])

    return K

if __name__ == "__main__":
    rospy.init_node('convert_sc2wc')
    pub = rospy.Publisher('object_coord', Point, queue_size=10)
    sub1 = rospy.Subscriber('Detected_Object_Position', Point, queue_size=10)
    sub2 = rospy.Subscriber('Object_Depth', float, queue_size=10)
    # カメラの座標
    cam_coord = [392, 336, 234]
    # カメラの回転角度（カメラ座標におけるx軸、y軸、z軸での回転）
    cam_rot = [326, 41, 0]
    # カメラの視野角（水平方向）
    fov = 86
    # スクリーンの画素数（横）
    pw = 1280
    # スクリーンの画素数（縦）
    ph = 720
    # カメラ情報（内部パラメータ）
    cam_info = (fov, pw, ph)

    t = cam_coord
    R = calc_R(*cam_rot)
    K = calc_K(*cam_info)

    cs = [0, 0]
    depth = 0
    cw = convert_uvz_to_xyz(cs[0], cs[1], depth, R, t, K)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

