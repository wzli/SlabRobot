#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.8)

robot = p.loadURDF("urdf/robot.urdf")
joints = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
legs = [joint[0] for joint in joints if joint[1].endswith(b"leg")]
wheels = [joint[0] for joint in joints if joint[1].endswith(b"wheel")]

# start with legs folded
for leg in legs:
    p.resetJointState(robot, leg, -np.pi, 0)

reset_button = p.addUserDebugParameter("reset",1,0,0)
MAX_SPEED = 10
speed_param = p.addUserDebugParameter("speed", -MAX_SPEED, MAX_SPEED, 0)
spin_param = p.addUserDebugParameter("spin", -MAX_SPEED, MAX_SPEED, 0)

#0 front right
#1 front left
#2 back right
#3 back left

def set_wheel_speed(v, w = 0):
    to_wheel_speeds = np.array([[1,1], [-1,1], [1,1], [-1,1]])
    p.setJointMotorControlArray(robot, wheels, p.VELOCITY_CONTROL, targetVelocities = to_wheel_speeds.dot([v, w]).clip(-100 * MAX_SPEED, 100 * MAX_SPEED))

def fold():
    p.setJointMotorControlArray(robot, legs, p.POSITION_CONTROL, targetPositions = [-np.pi, -np.pi], positionGains = [0.01, 0.01])
    set_wheel_speed(p.readUserDebugParameter(speed_param), p.readUserDebugParameter(spin_param))

def rover():
    p.setJointMotorControlArray(robot, legs, p.POSITION_CONTROL, targetPositions = [np.pi/2, -np.pi/2], positionGains = [0.004, 0.002])
    set_wheel_speed(p.readUserDebugParameter(speed_param), p.readUserDebugParameter(spin_param))

def stand():
    if stand.iteration == 0:
        pos = (0, 0, 1) # x y z
        orn = (1, 0, 0, 1) # qx qy qz qw
        p.resetBasePositionAndOrientation(robot, pos, orn)
        p.resetJointState(robot, legs[0], 0, 0)
        stand.front_leg = p.addUserDebugParameter("front_leg", -np.pi, np.pi, 0)
        stand.back_leg = p.addUserDebugParameter("back_leg", -np.pi, np.pi, -np.pi)

        stand.P = p.addUserDebugParameter("P", 0, 10000, 300)
        stand.I = p.addUserDebugParameter("I", 0, 0.001, 0.0001)
        stand.D = p.addUserDebugParameter("D", 0, 0.1, 0.05)
        stand.init = False

    stand.iteration += 1
    if stand.iteration < 250:
        return

    #p.setJointMotorControlArray(robot, legs, p.POSITION_CONTROL, targetPositions = [p.readUserDebugParameter(stand.front_leg), p.readUserDebugParameter(stand.back_leg)], positionGains = [0.01, 0.01])
    p.setJointMotorControlArray(robot, [legs[1]], p.POSITION_CONTROL, targetPositions = [p.readUserDebugParameter(stand.back_leg)], positionGains = [0.005])

    # set speed based on tilt feedback to maintain balance
    pos, orn = p.getBasePositionAndOrientation(robot)
    tilt_feedback = np.tan(p.getEulerFromQuaternion(orn)[0] - np.pi/2)
    tilt_error = stand.tilt_desired - tilt_feedback
    tilt_error_derivative = tilt_error - stand.tilt_error
    stand.tilt_error = tilt_error
    speed = p.readUserDebugParameter(stand.P) * tilt_error #- p.readUserDebugParameter(stand.I) * stand.speed_integral + p.readUserDebugParameter(stand.D) * tilt_error_derivative
    speed_error = p.readUserDebugParameter(speed_param) - speed
    stand.speed_integral += speed_error
    set_wheel_speed(-speed, p.readUserDebugParameter(spin_param));
    print(tilt_feedback, speed, stand.speed_integral)
    p.setJointMotorControlArray(robot, [legs[0]], p.POSITION_CONTROL, targetPositions = [-p.readUserDebugParameter(stand.I) * stand.speed_integral - p.readUserDebugParameter(stand.D) * speed_error], positionGains = [0.01])

stand.iteration = 0
stand.tilt_desired = 0
stand.tilt_error = 0
stand.speed_integral = 0


controller = stand
reset_button_count = 0

for i in range (100000):
    p.stepSimulation()
    #time.sleep(1)
    time.sleep(1./240.)
    controller()
    if p.readUserDebugParameter(reset_button) > reset_button_count:
        reset_button_count += 1
        camera_info = p.getDebugVisualizerCamera()
        pos, orn = p.getBasePositionAndOrientation(robot)
        p.resetDebugVisualizerCamera(camera_info[10], camera_info[8], camera_info[9], pos)

p.disconnect()

