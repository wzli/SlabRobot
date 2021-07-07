#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

robot = p.loadURDF("urdf/robot.urdf")
joints = {p.getJointInfo(robot, i)[1]: i for i in range(p.getNumJoints(robot))}
legs = [joints[b"front_leg"], joints[b"back_leg"]]
wheels = [
    joints[b"front_left_wheel"],
    joints[b"front_right_wheel"],
    joints[b"back_left_wheel"],
    joints[b"back_right_wheel"],
]

# start with legs folded
for leg in legs:
    p.resetJointState(robot, leg, -np.pi, 0)

reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
MAX_SPEED = 10
speed_param = p.addUserDebugParameter("speed", -MAX_SPEED, MAX_SPEED, 0)
spin_param = p.addUserDebugParameter("spin", -MAX_SPEED, MAX_SPEED, 0)

# 0 front right
# 1 front left
# 2 back right
# 3 back left


def set_wheel_speed(v, w=0):
    to_wheel_speeds = np.array([[-1, 1], [1, 1], [-1, 1], [1, 1]])
    p.setJointMotorControlArray(
        robot,
        wheels,
        p.VELOCITY_CONTROL,
        targetVelocities=to_wheel_speeds.dot([v, w]).clip(
            -100 * MAX_SPEED, 100 * MAX_SPEED
        ),
    )


def slab():
    p.setJointMotorControlArray(
        robot,
        legs,
        p.POSITION_CONTROL,
        targetPositions=[-np.pi, -np.pi],
        positionGains=[0.01, 0.01],
    )
    set_wheel_speed(
        p.readUserDebugParameter(speed_param), p.readUserDebugParameter(spin_param)
    )


def table():
    p.setJointMotorControlArray(
        robot,
        legs,
        p.POSITION_CONTROL,
        targetPositions=[np.pi / 2, -np.pi / 2],
        positionGains=[0.004, 0.002],
    )
    set_wheel_speed(
        p.readUserDebugParameter(speed_param), p.readUserDebugParameter(spin_param)
    )


def balance():
    if balance.iteration == 0:
        pos = (0, 0, 1.2)  # x y z
        orn = (1, 0, 0, 1)  # qx qy qz qw
        p.resetBasePositionAndOrientation(robot, pos, orn)
        p.resetJointState(robot, legs[0], 0, 0)
        balance.front_leg = p.addUserDebugParameter("front_leg", -np.pi, np.pi, 0)
        balance.back_leg = p.addUserDebugParameter("back_leg", -np.pi, np.pi, -np.pi)

        balance.P = p.addUserDebugParameter("P", 0, 10000, 300)
        balance.I = p.addUserDebugParameter("I", 0, 0.001, 0.0001)
        balance.D = p.addUserDebugParameter("D", 0, 0.1, 0.05)
        balance.init = False

    balance.iteration += 1
    if balance.iteration < 100:
        return

    # p.setJointMotorControlArray(robot, legs, p.POSITION_CONTROL, targetPositions = [p.readUserDebugParameter(balance.front_leg), p.readUserDebugParameter(balance.back_leg)], positionGains = [0.01, 0.01])
    p.setJointMotorControlArray(
        robot,
        [legs[1]],
        p.POSITION_CONTROL,
        targetPositions=[p.readUserDebugParameter(balance.back_leg)],
        positionGains=[0.005],
    )

    # set speed based on tilt feedback to maintain balance
    pos, orn = p.getBasePositionAndOrientation(robot)
    tilt_feedback = np.tan(p.getEulerFromQuaternion(orn)[0] - np.pi / 2)
    tilt_error = balance.tilt_desired - tilt_feedback
    tilt_error_derivative = tilt_error - balance.tilt_error
    balance.tilt_error = tilt_error
    speed = (
        p.readUserDebugParameter(balance.P) * tilt_error
    )  # - p.readUserDebugParameter(balance.I) * balance.speed_integral + p.readUserDebugParameter(balance.D) * tilt_error_derivative
    speed_error = p.readUserDebugParameter(speed_param) - speed
    balance.speed_integral += speed_error
    set_wheel_speed(-speed, p.readUserDebugParameter(spin_param))
    print(tilt_feedback, speed, balance.speed_integral)
    p.setJointMotorControlArray(
        robot,
        [legs[0]],
        p.POSITION_CONTROL,
        targetPositions=[
            p.readUserDebugParameter(balance.I) * balance.speed_integral
            + p.readUserDebugParameter(balance.D) * speed_error
        ],
        positionGains=[0.01],
    )


balance.iteration = 0
balance.tilt_desired = 0
balance.tilt_error = 0
balance.speed_integral = 0

controller = balance
reset_button_count = 0

for i in range(100000):
    p.stepSimulation()
    # time.sleep(1)
    time.sleep(1.0 / 240.0)
    controller()
    if p.readUserDebugParameter(reset_button) > reset_button_count:
        reset_button_count += 1
        camera_info = p.getDebugVisualizerCamera()
        pos, orn = p.getBasePositionAndOrientation(robot)
        p.resetDebugVisualizerCamera(
            camera_info[10], camera_info[8], camera_info[9], pos
        )

p.disconnect()
