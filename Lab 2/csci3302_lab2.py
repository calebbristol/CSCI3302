"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor
import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 0

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# These are for updating the pose values in the odometry step
theta_dot = 0
x_dot = 0
y_dot = 0

# Used For Looping Mechanism
starting_line = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 2
CENTER_IDX = 1
RIGHT_IDX = 0

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1265 # TODO: To be filled in with ePuck wheel speed in m/s
MAX_SPEED = 6.28
Wheel_Radius = EPUCK_MAX_WHEEL_SPEED/MAX_SPEED

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0 # TODO: Initialize variable for left speed
vR = 0 # TODO: Initialize variable for right speed

# For Part 1: Keeps Track of Time
#Time_Spent = 0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:



    # Part 1: Find Maximum Speed
    #if(Time_Spent < 2000):
    #    vL = MAX_SPEED
    #    vR = MAX_SPEED
    #else:
    #    vL = 0
    #    vR = 0
    #Time_Spent += SIM_TIMESTEP    
    #print(Time_Spent)
    
    
    
    
    
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    #print(gsr) # TODO: Uncomment to see the ground sensor values!
    # As read from gsr, the sensors read about 850 for white, 400 for dark
    #
    # Can Safely Settle on This Conclusion
    # gsr[i] < 550: Line Detected
    # gsr[i] > 550: No Line Detected
    #


    # Hints: 
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    
    # TODO: Insert Line Following Code Here                
    
    if gsr[1] < 450:
        # Go Forwards
        vL = MAX_SPEED
        vR = MAX_SPEED
    elif gsr[0] < 750:
        # Rotate Counter Clockwise
        vR = 0.25*MAX_SPEED
        vL = -0.25*MAX_SPEED
    elif gsr[2] < 750:
        # Rotate Clockwise
        vR = -0.25*MAX_SPEED
        vL = 0.25*MAX_SPEED
    else:
        # Rotate Counter Clockwise
        vR = 0.25*MAX_SPEED
        vL = -0.25*MAX_SPEED
    
    
    
    
    # TODO: Call update_odometry Here
    
    # Hints:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second. 
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    # Update Theta
    theta_dot = (vR - vL) * Wheel_Radius / EPUCK_AXLE_DIAMETER
    # Update X
    x_dot = math.cos(pose_theta) * (vR + vL) * Wheel_Radius / 2
    # Update Y
    y_dot = math.sin(pose_theta) * (vR + vL) * Wheel_Radius / 2
    
    # Update Pose
    pose_theta += theta_dot * SIM_TIMESTEP/1000
    pose_x += x_dot * SIM_TIMESTEP/1000
    pose_y += y_dot * SIM_TIMESTEP/1000
    
    # Keep theta within 360 degree bounds
    pose_theta %= (2*math.pi)
    
    
    # TODO: Insert Loop Closure Code Here
    
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last 
    # for best results
    
    # Starting Line Flag
    if gsr[0] < 450 and gsr[1] < 450 and gsr[2] < 450:
        if pose_theta < math.pi/2 or pose_theta > math.pi*7/4:
            starting_line = 1
    else:
        starting_line = 0
    
    # Reset Pose if Starting Line Flagged
    if starting_line == 1:
        pose_theta = 0
        pose_x = 0
        pose_y = 0
    
    
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)