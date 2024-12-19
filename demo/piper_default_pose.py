#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
from typing import (
    Optional,
)
import time
from piper_sdk import *
import csv
import signal
import sys
from datetime import datetime

# def enable_fun(piper:C_PiperInterface):
#     '''
#     使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
#     '''
#     enable_flag = False
#     # 设置超时时间（秒）
#     timeout = 5
#     # 记录进入循环前的时间
#     start_time = time.time()
#     elapsed_time_flag = False
#     while not (enable_flag):
#         elapsed_time = time.time() - start_time
#         print("--------------------")
#         enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
#             piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
#             piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
#             piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
#             piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
#             piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
#         print("使能状态:",enable_flag)
#         piper.EnableArm(7)
#         piper.GripperCtrl(0,1000,0x01, 0)
#         print("--------------------")
#         # 检查是否超过超时时间
#         if elapsed_time > timeout:
#             print("超时....")
#             elapsed_time_flag = True
#             enable_flag = True
#             break
#         time.sleep(1)
#         pass
#     if(elapsed_time_flag):
#         print("程序自动使能超时,退出程序")
#         exit(0)

def enable_fun(piper:C_PiperInterface, enable:bool):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    loop_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (loop_flag):
        elapsed_time = time.time() - start_time
        print(f"--------------------")
        enable_list = []
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
        if(enable):
            enable_flag = all(enable_list)
            piper.EnableArm(7)
            piper.GripperCtrl(0,1000,0x01, 0)
        else:
            enable_flag = any(enable_list)
            piper.DisableArm(7)
            piper.GripperCtrl(0,1000,0x02, 0)
        print(f"使能状态: {enable_flag}")
        print(f"--------------------")
        if(enable_flag == enable):
            loop_flag = True
            enable_flag = True
        else: 
            loop_flag = False
            enable_flag = False
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print(f"超时....")
            elapsed_time_flag = True
            enable_flag = False
            loop_flag = True
            break
        time.sleep(0.5)
    resp = enable_flag
    print(f"Returning response: {resp}")
    return resp

if __name__ == "__main__":
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper, enable=True)
    # piper.DisableArm(7)
    piper.GripperCtrl(0,1000,0x01, 0)
    joint_factor = 57324.840764 #1000*180/3.14 is the factor for joint position, radians
    gripper_factor = 1e6 #1000*1000 is the factor for gripper position, micrometers
    position = [0,0,0,0,0,0,0]
    count = 0
    while count < 1000:
        position = [0,0,0,0,0,0,0]

        joint_0 = round(position[0]*joint_factor)
        joint_1 = round(position[1]*joint_factor)
        joint_2 = round(position[2]*joint_factor)
        joint_3 = round(position[3]*joint_factor)
        joint_4 = round(position[4]*joint_factor)
        joint_5 = round(position[5]*joint_factor)
        joint_6 = round(position[6]*gripper_factor)
        # piper.MotionCtrl_1()
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        time.sleep(0.005)
        count += 1
    flag = enable_fun(piper=piper, enable=False)
    if(flag == True):
        print("失能成功!!!!")
        exit(0)