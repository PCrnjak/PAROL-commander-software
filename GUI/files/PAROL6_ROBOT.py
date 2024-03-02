# This file acts as configuration file for robot you are using
# It works in conjustion with configuration file from robotics toolbox

from swift import Swift
import spatialmath.base.symbolic as sym
from roboticstoolbox import ETS as ET
from roboticstoolbox import *
import roboticstoolbox as rtb
from spatialmath import *
from spatialgeometry import *
from math import pi
import numpy as np
import time
import random

Joint_num = 6 # Number of joints
Microstep = 32
steps_per_revolution=200
degree_per_step_constant = 360/(32*200) 
radian_per_step_constant = (2*pi) / (32*200)
radian_per_sec_2_deg_per_sec_const = 360/ (2*np.pi)
deg_per_sec_2_radian_per_sec_const = (2*np.pi) / 360

# robot length values (metres)
a1 = 110.50 / 1000
a2 = 23.42 / 1000
a3 = 180 / 1000
a4 = 43.5 / 1000
a5 = 176.35 / 1000
a6 = 62.8 / 1000
a7 = 45.25 / 1000

alpha_DH = [-pi / 2,pi,pi/2,-pi/2,pi/2,pi]

robot = DHRobot(
    [
        RevoluteDH(d=a1, a=a2, alpha=alpha_DH[0]),
        RevoluteDH(a=a3,d = 0,alpha=alpha_DH[1]),
        RevoluteDH(alpha= alpha_DH[2], a= -a4),
        RevoluteDH(d=-a5, a=0, alpha=alpha_DH[3]),
        RevoluteDH(a=0,d=0,alpha=alpha_DH[4]),
        RevoluteDH(alpha=alpha_DH[5], a = -a7,d = -a6),
    ],
    name="PAROL6",
)
#print(robot.isspherical())
#pyplot = rtb.backends.PyPlot()

# in degrees
Joints_standby_position_degree = np.array([0,-90,180,0,0,180]) 
# in radians
Joints_standby_position_radian = [np.deg2rad(angle) for angle in Joints_standby_position_degree]

# values you get after homing robot and moving it to its most left and right sides
# In degrees
Joint_limits_degree =[[-123.046875,123.046875], [-145.0088,-3.375], [107.866,287.8675], [-105.46975,105.46975], [-90,90], [0,360]] 

# in radians
Joint_limits_radian = []
for limits in Joint_limits_degree:
    radian_limits = [np.deg2rad(angle) for angle in limits]
    Joint_limits_radian.append(radian_limits)

# Reduction ratio we have on our joints
Joint_reduction_ratio = [6.4, 20, 20*(38/42) , 4, 4, 10] 

# min and max jog speeds. Usually slower from real maximal speeds
Joint_max_jog_speed = [1500, 3000, 3600, 7000, 7000, 18000]
Joint_min_jog_speed = [100,100,100,100,100,100]

# LINEAR CARTESIAN JOG MAX MIN SPEED IN METERS PER SECOND
Cartesian_linear_velocity_min_JOG = 0.002
Cartesian_linear_velocity_max_JOG = 0.06

# LINEAR CARTESIAN MAX MIN SPEED IN METERS PER SECOND
Cartesian_linear_velocity_min = 0.002
Cartesian_linear_velocity_max = 0.06

# LINEAR CARTESIAN MAX MIN ACC IN METERS PER SECOND²
Cartesian_linear_acc_min = 0.002
Cartesian_linear_acc_max = 0.06

# ANGULAR CARTESIAN JOG MAX MIN SPEED IN DEGREES PER SECOND
Cartesian_angular_velocity_min = 0.7
Cartesian_angular_velocity_max = 25

Joint_max_speed = [6500,18000,20000,22000,22000,22000] # max speed in STEP/S used
Joint_min_speed = [100,100,100,100,100,100] # min speed in STEP/S used 

Joint_max_acc = 32000 # max acceleration in RAD/S²
Joint_min_acc = 100 # min acceleration in RAD/S²

Cart_lin_velocity_limits = [[-100,100],[-100,100],[-100,100]]
Cart_ang_velocity_limits = [[-100,100],[-100,100],[-100,100]]


Commands_list = [ "Input","Output","Dummy","Begin","Home","Delay","End","Loop","MoveJoint","MovePose","SpeedJoint","MoveCart",
                 "MoveCart","MoveCartRelTRF","Gripper","Gripper_cal"]

Commands_list_true = [item + "()" for item in Commands_list]

# 360 / (200 * 32) = 0.05625
def DEG2STEPS(Degrees, index):
    Steps = Degrees / degree_per_step_constant * Joint_reduction_ratio[index]
    return Steps

Joint_limits_steps =[[DEG2STEPS(Joint_limits_degree[0][0],0),DEG2STEPS(Joint_limits_degree[0][1],0)],
                      [DEG2STEPS(Joint_limits_degree[1][0],1),DEG2STEPS(Joint_limits_degree[1][1],1)],
                      [DEG2STEPS(Joint_limits_degree[2][0],2),DEG2STEPS(Joint_limits_degree[2][1],2)],
                      [DEG2STEPS(Joint_limits_degree[3][0],3),DEG2STEPS(Joint_limits_degree[3][1],3)],
                      [DEG2STEPS(Joint_limits_degree[4][0],4),DEG2STEPS(Joint_limits_degree[4][1],4)],
                      [DEG2STEPS(Joint_limits_degree[5][0],5),DEG2STEPS(Joint_limits_degree[5][1],5)]]
Joint_limits_steps = [[int(i[0]),int(i[1])] for i in Joint_limits_steps]


def STEPS2DEG(Steps,index):
    Degrees = Steps * degree_per_step_constant / Joint_reduction_ratio[index]
    return Degrees

def RAD2STEPS(Rads,index):
    deg = np.rad2deg(Rads)
    steps = DEG2STEPS(deg,index)
    return steps

def STEPS2RADS(Steps,index):
    deg = STEPS2DEG(Steps,index)
    rads = np.deg2rad(deg)
    return rads

def RAD2DEG(radian):
    return np.rad2deg(radian)

def DEG2RAD(degree):
    return np.deg2rad(degree)

def SPEED_STEPS2DEG(Steps_per_second,index):

    '''     Transform true RADS/S to true RPM.
    Both these values are true values at witch MOTORS SPIN  '''

    degrees_per_step = degree_per_step_constant / Joint_reduction_ratio[index]
    degrees_per_second = Steps_per_second * degrees_per_step
    return degrees_per_second

def SPEED_DEG2STEPS(Deg_per_second,index):
    steps_per_second = Deg_per_second / degree_per_step_constant * Joint_reduction_ratio[index]
    return steps_per_second

def SPEED_STEP2RAD(Steps_per_second,index):
    degrees_per_step = radian_per_step_constant / Joint_reduction_ratio[index]
    rad_per_second = Steps_per_second * degrees_per_step
    return rad_per_second

def SPEED_RAD2STEP(Rad_per_second,index):
    steps_per_second = Rad_per_second / radian_per_step_constant * Joint_reduction_ratio[index]
    return steps_per_second

def RAD_SEC_2_DEG_SEC(rad_per_sec):
    return rad_per_sec * radian_per_sec_2_deg_per_sec_const

def DEG_SEC_2_RAD_SEC(deg_per_sec):
    return deg_per_sec * deg_per_sec_2_radian_per_sec_const


def extract_from_can_id(can_id):
    # Extracting ID2 (first 4 MSB)
    id2 = (can_id >> 7) & 0xF

    # Extracting CAN Command (next 6 bits)
    can_command = (can_id >> 1) & 0x3F

    # Extracting Error Bit (last bit)
    error_bit = can_id & 0x1
    
    return id2, can_command, error_bit


def combine_2_can_id(id2, can_command, error_bit):
    # Combine components into an 11-bit CAN ID
    can_id = 0

    # Add ID2 (first 4 MSB)
    can_id |= (id2 & 0xF) << 7

    # Add CAN Command (next 6 bits)
    can_id |= (can_command & 0x3F) << 1

    # Add Error Bit (last bit)
    can_id |= (error_bit & 0x1)

    return can_id

# Fuse bitfield list to byte
def fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])

# Splits byte to bitfield list
def split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)] 
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]


if __name__ == "__main__":
    """
    print(DEG2STEPS(180,2))
    print(STEPS2DEG(57905,2))
    print(RAD2STEPS(pi,5))
    print(STEPS2RADS(32000,5))
    print(SPEED_STEPS2DEG(1000,5))
    print(SPEED_STEP2RAD(1000,5))
    print(Joint_limits_radian)
    print(Joints_standby_position_radian)
    print(Joint_limits_steps)
    print(Joint_limits_radian)
    print(DEG2STEPS(-62.5,1))
    """

    J0_var = STEPS2RADS(1,0)
    J1_var = STEPS2RADS(1,1)
    J2_var = STEPS2RADS(1,2)
    J3_var = STEPS2RADS(1,3)
    J4_var = STEPS2RADS(1,4)
    J5_var = STEPS2RADS(1,5)


    print("Joint 1 smallest step:",RAD2DEG(J0_var))
    print("Joint 2 smallest step:",RAD2DEG(J1_var))
    print("Joint 3 smallest step:",RAD2DEG(J2_var))
    print("Joint 4 smallest step:",RAD2DEG(J3_var))
    print("Joint 5 smallest step:",RAD2DEG(J4_var))
    print("Joint 6 smallest step:",RAD2DEG(J5_var))
    print("rad 2 step:",SPEED_RAD2STEP(-2.948504399390715 / 2,5))
    print("standby radian is",Joints_standby_position_radian)

    test = RAD2STEPS(0.0001,5)
    print(test)

    #robot.ikine_LMS()

