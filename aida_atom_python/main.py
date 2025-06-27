#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import binascii
import time
import threading
import configparser
from data_filter import DataFilter
from Robotic_Arm.rm_robot_interface import *

Arm_dof = 7
master_dof = 7

l_ip = "169.254.128.18"
r_ip = "169.254.128.19"

l_port = "/dev/rmUSB0"
r_port = "/dev/rmUSB1"

l_data_filter = DataFilter()
r_data_filter = DataFilter()

# 平滑滤波深度
filter_depth = 10
# 全局变量用于存储关节数据
l_joints = []
r_joints = []
l_grip = 0
r_grip = 0

# 线程锁
data_lock = threading.Lock()

def read_config():
    config = configparser.ConfigParser()
    config.read('config.ini')
    global Arm_dof,master_dof,l_ip,r_ip,l_port,r_port,filter_depth
    try:
        Arm_dof = int(config.get('DOF', 'slave_dof'))
        master_dof = int(config.get('DOF', 'master_dof'))

        l_ip = config.get('IPAddresses', 'left_arm_ip')
        r_ip = config.get('IPAddresses', 'right_arm_ip')

        l_port = config.get('SerialPorts', 'master_port')
        r_port = config.get('SerialPorts', 'slave_port')

        filter_depth = config.get('DataFilter', 'depth')
    except:
        return False
    return True

def arm_conf():

    baudrate = 460800
    hex_data = "55 AA 02 00 00 67"

    l_ser = serial.Serial(l_port, baudrate, timeout=0)
    r_ser = serial.Serial(r_port, baudrate, timeout=0)

    bytes_to_send = binascii.unhexlify(hex_data.replace(" ", ""))
    l_ser.write(bytes_to_send)
    r_ser.write(bytes_to_send)
    time.sleep(1)

    return l_ser, r_ser, bytes_to_send

def bytes_to_signed_int(byte_data):
    value = int.from_bytes(byte_data, byteorder='little', signed=True)
    return value

def get_Joint(hex_received):
    # global master_dof
    if len(hex_received) == 94:
        # master_dof = 7
        J1 = hex_received[14:22]
        J1_byte_data = bytearray.fromhex(J1)
        Joint1 = bytes_to_signed_int(J1_byte_data) / 10000.0

        J2 = hex_received[24:32]
        J2_byte_data = bytearray.fromhex(J2)
        Joint2 = bytes_to_signed_int(J2_byte_data) / 10000.0

        J3 = hex_received[34:42]
        J3_byte_data = bytearray.fromhex(J3)
        Joint3 = bytes_to_signed_int(J3_byte_data) / 10000.0

        J4 = hex_received[44:52]
        J4_byte_data = bytearray.fromhex(J4)
        Joint4 = bytes_to_signed_int(J4_byte_data) / 10000.0

        J5 = hex_received[54:62]
        J5_byte_data = bytearray.fromhex(J5)
        Joint5 = bytes_to_signed_int(J5_byte_data) / 10000.0

        J6 = hex_received[64:72]
        J6_byte_data = bytearray.fromhex(J6)
        Joint6 = bytes_to_signed_int(J6_byte_data) / 10000.0

        J7 = hex_received[74:82]
        J7_byte_data = bytearray.fromhex(J7)
        Joint7 = bytes_to_signed_int(J7_byte_data) / 10000.0

        G7 = hex_received[84:92]
        G7_byte_data = bytearray.fromhex(G7)
        Grasp = bytes_to_signed_int(G7_byte_data)

        Joints = [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, Joint7]
        Gripper = Grasp

    elif len(hex_received) == 84:
        # master_dof = 6
        J1 = hex_received[14:22]
        J1_byte_data = bytearray.fromhex(J1)
        Joint1 = bytes_to_signed_int(J1_byte_data) / 10000.0

        J2 = hex_received[24:32]
        J2_byte_data = bytearray.fromhex(J2)
        Joint2 = bytes_to_signed_int(J2_byte_data) / 10000.0

        J3 = hex_received[34:42]
        J3_byte_data = bytearray.fromhex(J3)
        Joint3 = bytes_to_signed_int(J3_byte_data) / 10000.0

        J4 = hex_received[44:52]
        J4_byte_data = bytearray.fromhex(J4)
        Joint4 = bytes_to_signed_int(J4_byte_data) / 10000.0

        J5 = hex_received[54:62]
        J5_byte_data = bytearray.fromhex(J5)
        Joint5 = bytes_to_signed_int(J5_byte_data) / 10000.0

        J6 = hex_received[64:72]
        J6_byte_data = bytearray.fromhex(J6)
        Joint6 = bytes_to_signed_int(J6_byte_data) / 10000.0

        G6 = hex_received[74:82]
        G6_byte_data = bytearray.fromhex(G6)
        Grasp = bytes_to_signed_int(G6_byte_data)

        Joints = [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
        Gripper = Grasp
    else:
        raise Exception("主臂数据错误")

    return Joints, Grasp

def get_master_act(l_ser, r_ser, byte_send):
    l_ser.write(byte_send)
    r_ser.write(byte_send)
    l_bytes_received = l_ser.read(l_ser.inWaiting())
    r_bytes_received = r_ser.read(r_ser.inWaiting())
    l_hex_received = binascii.hexlify(l_bytes_received).decode('utf-8').upper()
    r_hex_received = binascii.hexlify(r_bytes_received).decode('utf-8').upper()
    l_joints, l_grip = get_Joint(l_hex_received)
    r_joints, r_grip = get_Joint(r_hex_received)

    return l_joints, r_joints, l_grip, r_grip

def filter_joints(joints, data_filter):
    joint_filter = [0 for _ in range(Arm_dof)]
    joint_filter[0] = data_filter.Refilter(0, joints[0], 10)
    joint_filter[1] = data_filter.Refilter(1, joints[1], 10)

    if Arm_dof == master_dof:
        for i in range(2,Arm_dof):
            joint_filter[i] = data_filter.Refilter(i, joints[i], 10)
    elif Arm_dof < master_dof:
        for i in range(2,Arm_dof):
            joint_filter[i] = data_filter.Refilter(i, joints[i + 1], 10)
    elif Arm_dof > master_dof:
        for i in range(2,master_dof):
            joint_filter[i+1] = data_filter.Refilter(i+1, joints[i], 10)

    return joint_filter

def get_master_joints_thread(l_ser, r_ser, byte_send):
    global l_joints, r_joints, l_grip, r_grip
    while True:
        l_joints, r_joints, l_grip, r_grip = get_master_act(l_ser, r_ser, byte_send)
        l_joints = filter_joints(l_joints, l_data_filter)
        r_joints = filter_joints(r_joints, r_data_filter)
        time.sleep(0.02)

def move_left_arm_thread():
    l_arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
    l_handle = l_arm.rm_create_robot_arm("169.254.128.18", 8080)
    if l_handle is None:
        print("左臂初始化失败")
        return

    global l_joints
    # 等待初始关节角度
    while not l_joints:
        time.sleep(0.01)

    # 使用 movej 同步到初始姿态
    l_arm.rm_movej(l_joints, 5, 0, 0, 1)
    time.sleep(2)  # 等待机械臂移动完成

    while True:
        with data_lock:
            joints = l_joints
        if joints:
            l_arm.rm_movej_canfd(joints, True, 0, 0, 0)
        time.sleep(0.02)

def move_right_arm_thread():
    r_arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
    r_handle = r_arm.rm_create_robot_arm("169.254.128.19", 8080)
    if r_handle is None:
        print("右臂初始化失败")
        return

    global r_joints
    # 等待初始关节角度
    while not r_joints:
        time.sleep(0.01)

    # 使用 movej 同步到初始姿态
    r_arm.rm_movej(r_joints, 5, 0, 0, 1)
    time.sleep(2)  # 等待机械臂移动完成

    while True:
        with data_lock:
            joints = r_joints
        if joints:
            r_arm.rm_movej_canfd(joints, True, 0, 0, 0)
        time.sleep(0.02)

def main():

    if read_config() == False:
        print("read config wrong!!!")
        return

    l_ser, r_ser, byte_send = arm_conf()
    time.sleep(2)
    print(l_ser, '\n', r_ser)

    # 启动获取关节角度的线程
    get_master_joints_thread_obj = threading.Thread(target=get_master_joints_thread, args=(l_ser, r_ser, byte_send), daemon=True)
    get_master_joints_thread_obj.start()

    # 启动左臂和右臂的控制线程
    move_left_arm_thread_obj = threading.Thread(target=move_left_arm_thread, daemon=True)
    move_right_arm_thread_obj = threading.Thread(target=move_right_arm_thread, daemon=True)

    move_left_arm_thread_obj.start()
    move_right_arm_thread_obj.start()

    # 主线程等待线程执行
    get_master_joints_thread_obj.join()
    move_left_arm_thread_obj.join()
    move_right_arm_thread_obj.join()

if __name__ == '__main__':
    main()