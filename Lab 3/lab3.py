"""lab3 controller."""
# Copyright University of Colorado Boulder 2022
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.16 # [m]
WHEEL_RADIUS = MAX_SPEED_MS/MAX_SPEED


MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = -8
pose_y     = -5
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

#goal1 = [1.5, -0.75]
#goal2 = [4, 1]

goal1 = [-6.5, -5.75]
goal2 = [-4.25, -4]
goal3 = [-3.05, -3.96]

goalList = [goal1, goal2, goal3]
g_ind = 0
final_goal = 0

while robot.step(timestep) != -1:
    # STEP 2.1: Calculate error with respect to current and goal position
    print(g_ind)
    if g_ind >= len(goalList)-1:
       final_goal = 1
       g_ind = len(goalList) - 1
       
    goal_current = goalList[g_ind]
    
    x_g = goal_current[0]
    y_g = goal_current[1]
    
    if final_goal == 0:
        goal_next = goalList[g_ind + 1]
    
        x_g2 = goal_next[0]
        y_g2 = goal_next[1]
        
        theta_g = math.atan2((y_g2 - y_g),(x_g2 - x_g))
    else:
        theta_g = 0   
    
    rho = math.sqrt((pose_x - x_g) ** 2 + (pose_y - y_g) ** 2)
    
    alpha = math.atan2((y_g - pose_y),(x_g - pose_x)) - pose_theta
    
    heading = theta_g - pose_theta
    
    pass   
    
    # STEP 2.2: Feedback Controller
    
    p1 = 1
    p2 = 10
    p3 = 0
    
    dX = p1*rho

    dTheta = p2*alpha + p3*heading
    
    pass
    
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    
    vL = (dX - ((dTheta*AXLE_LENGTH)/2))/WHEEL_RADIUS
    vR = (dX + ((dTheta*AXLE_LENGTH)/2))/WHEEL_RADIUS
    
    pass
    
    # STEP 2.3: Proportional velocities
    
    if vL > vR:
       left_speed = MAX_SPEED
       right_speed = (vR/vL)*MAX_SPEED
    elif vR > vL:
        right_speed = MAX_SPEED
        left_speed = (vL/vR)*MAX_SPEED
    else:
        right_speed = MAX_SPEED
        left_speed = MAX_SPEED
    
    vL = left_speed
    vR = right_speed    
    
    pass

    # STEP 2.4: Clamp wheel speeds
    
    angleBound = 3.14/6
    
    if rho < .03 and heading < angleBound and heading > -angleBound:
        vL = 0
        vR = 0
        g_ind += 1
    pass


    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line
    
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)

    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)

    pose_theta += (distR-distL)/AXLE_LENGTH
    
    print(pose_x)
    print(pose_y)

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)    
    