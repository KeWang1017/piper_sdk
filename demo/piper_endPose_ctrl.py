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

# Global variables
data_rows = []
csv_filename = f"arm_poses_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    print('\nSaving data and exiting...')
    save_to_csv()
    sys.exit(0)

# Function to save data to CSV
def save_to_csv():
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'RX', 'RY', 'RZ', 'grippers_angle', 'grippers_effort'])
        # Write all stored data
        writer.writerows(data_rows)
    print(f"Data saved to {csv_filename}")

# Register the signal handler
# signal.signal(signal.SIGINT, signal_handler)

def enable_fun(piper:C_PiperInterface):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("使能状态:",enable_flag)
        piper.EnableArm(7)
        piper.GripperCtrl(0,1000,0x01, 0)
        print("--------------------")
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if(elapsed_time_flag):
        print("程序自动使能超时,退出程序")
        exit(0)

if __name__ == "__main__":
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    # piper.MotionCtrl_1(0x02,0,0)#恢复
    # piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
    # piper.DisableArm(7)
    piper.GripperCtrl(0,1000,0x01, 0)
    endpose_factor = 1e6 #1000*1000 is the factor for joint position, radians
    gripper_factor = 1e6 #1000*1000 is the factor for gripper position, micrometers
    position = [0,0,0,0,0,0,0]
    count = 0
    while True:
        pose = piper.GetArmEndPoseMsgs()
        gripper_pose = piper.GetArmGripperMsgs()
        # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        # # breakpoint()
        
        # # Store the data
        # data_rows.append([
        #     timestamp,
        #     pose.end_pose.X_axis,
        #     pose.end_pose.Y_axis,
        #     pose.end_pose.Z_axis,
        #     pose.end_pose.RX_axis,
        #     pose.end_pose.RY_axis,
        #     pose.end_pose.RZ_axis,
        #     gripper_pose.gripper_state.grippers_angle,
        #     gripper_pose.gripper_state.grippers_effort
        # ])
        
        # Optional: Print the data as before
        # print(pose)
        
        import time
        count  = count + 1
        # print(count)
        if(count == 0):
            print("1-----------")
            position = [0.07,0,0.22,0,0.08,0,0]
        elif(count == 800):
            print("2-----------")
            position = [0.2,0.0,0.4,0.0,0.08,0.0,0.08] # 0.08 is maximum gripper position
        elif(count == 1600):
            print("1-----------")
            position = [0.07,0,0.22,0,0.08,0,0]
            count = 0
        
        X = round(position[0]*endpose_factor)
        Y = round(position[1]*endpose_factor)
        Z = round(position[2]*endpose_factor)
        RX = round(position[3]*endpose_factor)
        RY = round(position[4]*endpose_factor)
        RZ = round(position[5]*endpose_factor)
        Gripper = round(position[6]*gripper_factor)
        # piper.MotionCtrl_1()
        piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
        piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
        piper.GripperCtrl(abs(Gripper), 1000, 0x01, 0)
        piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
        time.sleep(0.005)
        pass