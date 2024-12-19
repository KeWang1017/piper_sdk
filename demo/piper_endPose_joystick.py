#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
from typing import (
    Optional,
)
import time
import numpy as np
from piper_sdk import *
import csv
import signal
import sys
from datetime import datetime
from joystick_expert import JoystickExpert,ControllerType


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


class Rate:
    def __init__(self, hz: float):
        self.period = 1.0 / hz
        self.last_time = time.perf_counter()
    
    def sleep(self):
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.last_time = time.perf_counter()


class JoystickIntervention():
    def __init__(self, controller_type=ControllerType.XBOX, gripper_enabled=True):
        self.gripper_enabled = gripper_enabled
        self.expert = JoystickExpert(controller_type=controller_type)
        self.left, self.right = False, False

    def action(self, action: np.ndarray) -> np.ndarray:
        """
        Input:
        - action: policy action
        Output:
        - action: joystick action if nonezero; else, policy action
        """
        deadzone = 0.003

        expert_a, buttons = self.expert.get_action()
        self.left, self.right = tuple(buttons)

        for i, a in enumerate(expert_a):
            if abs(a) <= deadzone:
                expert_a[i] = 0.0
        if abs(expert_a[0]) >= 0.003 and expert_a[1] >= 0.003 and expert_a[1] <= 0.005:
            expert_a[1] = 0.0
        expert_a[3:6] /= 2

        # if np.linalg.norm(expert_a) <= deadzone:
        #     expert_a = np.zeros_like(expert_a)

        if self.gripper_enabled:
            if self.left: # close gripper
                gripper_action = [0.0]
            elif self.right: # open gripper
                gripper_action = [0.08]
            else:
                gripper_action = [0.0]
            expert_a = np.concatenate((expert_a, gripper_action), axis=0)
        
        return expert_a
    
    def close(self):
        self.expert.close()


if __name__ == "__main__":
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(0,1000,0x01, 0)
    endpose_factor = 1e6 #1000*180/3.14 is the factor for joint position, radians
    gripper_factor = 1e6 #1000*1000 is the factor for gripper position, micrometers
    position = [0,0,0,0,0,0,0]
    rate = Rate(200)
    joystick = JoystickIntervention(controller_type=ControllerType.XBOX, gripper_enabled=True)

    count = 0
    while True:
        pose = piper.GetArmEndPoseMsgs()
        gripper_pose = piper.GetArmGripperMsgs()
        
        # Optional: Print the data as before
        # print(pose)
        
        count  = count + 1
        # print(count)
        if(count == 0):
            print("1-----------")
            position = [0.07,0,0.22,0,0.08,0,0]
        elif(count == 400):
            print("2-----------")
            position = [0.15,0.0,0.35,0.08,0.08,0.075,0.0] # 0.08 is maximum gripper position
        elif(count == 800):
            print("3-----------")
            # [0.255978, -0.021077, 0.310093, 0.16402, 0.032227, 0.155577, 0.0]
            position = [0.236107, 0.03445, 0.322808, 0.179081, 0.016596, 0.175193, 0.0]
        elif(count > 1000):
            print("4-----------")
            position = np.array([pose.end_pose.X_axis,pose.end_pose.Y_axis,pose.end_pose.Z_axis
                        ,pose.end_pose.RX_axis,pose.end_pose.RY_axis,pose.end_pose.RZ_axis
                        ,gripper_pose.gripper_state.grippers_angle])/endpose_factor

            action = joystick.action(position)
            position[:6] += action[:6]
            position[6] = action[6]
            print(action)
            print(position)
            # breakpoint()
        
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
        rate.sleep()
    
    joystick.close()