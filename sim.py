#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import numpy as np
import time
import json
import inputs, os, io, fcntl

import ctypes
from libslab.build import libslab_py


def ctype_to_dict(x):
    if isinstance(x, ctypes.Array):
        return [ctype_to_dict(element) for element in x]
    if not hasattr(x, "_fields_"):
        return x
    return {field: ctype_to_dict(getattr(x, field)) for field, _ in x._fields_}


def print_ctype(x):
    print(json.dumps(ctype_to_dict(x), indent=2))


class Simulation:
    def __init__(self):
        self.init_inputs()
        # pybullet setup
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # add user interface
        self.reset_button_count = 0
        self.reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
        self.center_button_count = 0
        self.center_button = p.addUserDebugParameter("center", 1, 0, 0)
        self.sim_rate = p.addUserDebugParameter("sim_rate", 0, 5, 1)
        self.incline_kp = p.addUserDebugParameter("Incline Kp", 0, 50, 30)
        self.speed_kp = p.addUserDebugParameter("Speed Kp", 0, 1.0, 0.6)
        self.speed_ki = p.addUserDebugParameter("Speed Ki", 0, 0.02, 0.003)
        # reset
        self.step_divider = 3
        self.reset()

    def init_inputs(self):
        # init inputs
        self.inputs = {}
        try:
            self.keyboard = inputs.devices.keyboards[0]
            self.keyboard.read()
        except PermissionError:
            print("No keyboard permission.")
            self.keyboard = None
        try:
            self.gamepad = inputs.devices.gamepads[0]
            self.gamepad._character_file = io.open(
                self.gamepad._character_device_path, "rb"
            )
            fd = self.gamepad._character_file.fileno()
            flag = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
        except IndexError:
            print("No gamepad found.")
            self.gamepad = None

    def reset(self):
        p.resetSimulation()
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.8)
        self.steps = 0
        self.prev_base_velocity = np.array([0, 0, 0])
        # load robot urdf
        self.robot = p.loadURDF("urdf/robot.urdf")
        self.joints = {
            p.getJointInfo(self.robot, i)[1]: i
            for i in range(p.getNumJoints(self.robot))
        }
        self.legs = [self.joints[b"front_leg"], self.joints[b"back_leg"]]
        self.wheels = [
            self.joints[b"front_left_wheel"],
            self.joints[b"front_right_wheel"],
            self.joints[b"back_left_wheel"],
            self.joints[b"back_right_wheel"],
        ]
        # add torque sensor to wheels
        for wheel in self.wheels:
            p.enableJointForceTorqueSensor(self.robot, wheel)
        self.slab = libslab_py.Slab()
        # start with legs folded
        for i, leg in enumerate(self.legs):
            p.resetJointState(self.robot, leg, -np.pi, 0)
            self.slab.input.leg_positions[i] = -np.pi
            self.slab.motors[i].input.position = -np.pi
        """
        # stand up orientation
        pos = (0, 0, 1.2)  # x y z
        orn = (1, 0, 0, 1)  # qx qy qz qw
        p.resetBasePositionAndOrientation(self.robot, pos, orn)
        """
        # set slab config
        self.slab.config.max_wheel_speed = 40  # rad/s
        self.slab.config.wheel_diameter = 0.165  # m
        self.slab.config.wheel_distance = 0.4  # m
        self.slab.config.body_length = 0.4  # m
        self.slab.config.leg_length = 0.35  # m
        self.slab.config.max_leg_position = np.pi  # rad
        self.slab.config.min_leg_position = -np.pi  # rad
        self.slab.config.imu_axis_remap[
            libslab_py.AXIS_REMAP_X
        ] = libslab_py.AXIS_REMAP_NEG_X
        self.slab.config.imu_axis_remap[
            libslab_py.AXIS_REMAP_Y
        ] = libslab_py.AXIS_REMAP_NEG_Y
        self.slab.config.imu_axis_remap[
            libslab_py.AXIS_REMAP_Z
        ] = libslab_py.AXIS_REMAP_Z
        self.slab.config.incline_p_gain = p.readUserDebugParameter(self.incline_kp)
        self.slab.config.speed_p_gain = p.readUserDebugParameter(self.speed_kp)
        self.slab.config.speed_i_gain = p.readUserDebugParameter(self.speed_ki)
        self.slab.config.ground_rise_threshold = 0.08  # m
        self.slab.config.ground_fall_threshold = 0.04  # m
        self.slab.config.joystick_threshold = 10  # 0 - 255
        # reset camera
        p.resetDebugVisualizerCamera(5, 50, -35, (0, 0, 0))

    def handle_center_button(self):
        button_count = p.readUserDebugParameter(self.center_button)
        if (
            button_count > self.center_button_count
            or self.inputs.get("BTN_SELECT", 0) == 1
        ):
            self.center_button_count = button_count
            camera_info = p.getDebugVisualizerCamera()
            pos, orn = p.getBasePositionAndOrientation(self.robot)
            p.resetDebugVisualizerCamera(
                camera_info[10], camera_info[8], camera_info[9], pos
            )

    def handle_reset_button(self):
        button_count = p.readUserDebugParameter(self.reset_button)
        if (
            button_count > self.reset_button_count
            or self.inputs.get("BTN_MODE", 0) == 1
        ):
            self.reset_button_count = button_count
            self.inputs["BTN_MODE"] = 0
            self.reset()

    def update_imu(self):
        # update orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        self.slab.imu.orientation.qx = orn[0]
        self.slab.imu.orientation.qy = orn[1]
        self.slab.imu.orientation.qz = orn[2]
        self.slab.imu.orientation.qw = orn[3]
        # TODO transform to local frame
        # update angular velocity
        lin_vel, ang_vel = p.getBaseVelocity(self.robot)
        self.slab.imu.angular_velocity.x = ang_vel[0]
        self.slab.imu.angular_velocity.y = ang_vel[1]
        self.slab.imu.angular_velocity.z = ang_vel[2]
        # TODO update linear acceleration
        self.slab.imu.linear_acceleration.x = lin_vel[0]
        self.slab.imu.linear_acceleration.y = lin_vel[1]
        self.slab.imu.linear_acceleration.z = lin_vel[2]
        """
        lin_vel = np.array(lin_vel)
        lin_acc = (lin_vel - self.prev_base_velocity) / self.step_divider
        self.prev_base_velocity = lin_vel
        self.slab.imu.linear_acceleration.x = lin_acc[0]
        self.slab.imu.angular_velocity.y = lin_acc[1]
        self.slab.imu.angular_velocity.z = lin_acc[2]
        """

    def update_motors(self):
        # update motor feedback
        joint_states = p.getJointStates(self.robot, self.legs + self.wheels)
        for i, state in enumerate(joint_states):
            self.slab.motors[i].estimate.position = state[0]
            self.slab.motors[i].estimate.velocity = state[1]
        self.slab.motors[libslab_py.MOTOR_ID_FRONT_LEFT_WHEEL].estimate.position *= -1
        self.slab.motors[libslab_py.MOTOR_ID_FRONT_LEFT_WHEEL].estimate.velocity *= -1
        self.slab.motors[libslab_py.MOTOR_ID_BACK_LEFT_WHEEL].estimate.position *= -1
        self.slab.motors[libslab_py.MOTOR_ID_BACK_LEFT_WHEEL].estimate.velocity *= -1
        # update wheel speed input
        p.setJointMotorControlArray(
            self.robot,
            self.wheels,
            p.VELOCITY_CONTROL,
            targetVelocities=[
                -self.slab.motors[libslab_py.MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity,
                self.slab.motors[libslab_py.MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity,
                -self.slab.motors[libslab_py.MOTOR_ID_BACK_LEFT_WHEEL].input.velocity,
                self.slab.motors[libslab_py.MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity,
            ],
        )
        # update leg position input
        p.setJointMotorControlArray(
            self.robot,
            self.legs,
            p.POSITION_CONTROL,
            targetPositions=[
                self.slab.motors[libslab_py.MOTOR_ID_FRONT_LEGS].input.position,
                self.slab.motors[libslab_py.MOTOR_ID_BACK_LEGS].input.position,
            ],
            positionGains=[0.01, 0.01],
        )

    def update_inputs(self):
        if self.keyboard:
            for event in self.keyboard.read():
                self.inputs[event.code] = event.state
        if self.gamepad:
            events = self.gamepad._do_iter()
            if events:
                for event in events:
                    self.inputs[event.code] = event.state
        self.slab.gamepad.buttons = (
            (self.inputs.get("BTN_SELECT", 0) << libslab_py.GAMEPAD_BUTTON_SELECT)
            | (self.inputs.get("BTN_THUMBL", 0) << libslab_py.GAMEPAD_BUTTON_L3)
            | (self.inputs.get("BTN_THUMBR", 0) << libslab_py.GAMEPAD_BUTTON_R3)
            | (self.inputs.get("BTN_START", 0) << libslab_py.GAMEPAD_BUTTON_START)
            | (self.inputs.get("BTN_DPAD_UP", 0) << libslab_py.GAMEPAD_BUTTON_UP)
            | (self.inputs.get("BTN_DPAD_RIGHT", 0) << libslab_py.GAMEPAD_BUTTON_RIGHT)
            | (self.inputs.get("BTN_DPAD_DOWN", 0) << libslab_py.GAMEPAD_BUTTON_DOWN)
            | (self.inputs.get("BTN_DPAD_LEFT", 0) << libslab_py.GAMEPAD_BUTTON_LEFT)
            | (min(self.inputs.get("ABS_Z", 0), 1) << libslab_py.GAMEPAD_BUTTON_L2)
            | (min(self.inputs.get("ABS_RZ", 0), 1) << libslab_py.GAMEPAD_BUTTON_R2)
            | (self.inputs.get("BTN_TL", 0) << libslab_py.GAMEPAD_BUTTON_L1)
            | (self.inputs.get("BTN_TR", 0) << libslab_py.GAMEPAD_BUTTON_R1)
            | (self.inputs.get("BTN_NORTH", 0) << libslab_py.GAMEPAD_BUTTON_TRIANGLE)
            | (self.inputs.get("BTN_EAST", 0) << libslab_py.GAMEPAD_BUTTON_CIRCLE)
            | (self.inputs.get("BTN_SOUTH", 0) << libslab_py.GAMEPAD_BUTTON_CROSS)
            | (self.inputs.get("BTN_WEST", 0) << libslab_py.GAMEPAD_BUTTON_SQUARE)
        )
        self.slab.gamepad.left_stick.x = self.inputs.get("ABS_X", 128) - 128
        self.slab.gamepad.left_stick.y = self.inputs.get("ABS_Y", 128) - 128
        self.slab.gamepad.right_stick.x = self.inputs.get("ABS_RX", 128) - 128
        self.slab.gamepad.right_stick.y = self.inputs.get("ABS_RY", 128) - 128
        self.slab.gamepad.left_trigger = self.inputs.get("ABS_Z", 0)
        self.slab.gamepad.right_trigger = self.inputs.get("ABS_RZ", 0)

    def update_slab(self):
        if self.steps % self.step_divider > 0:
            return
        self.slab.tick = self.steps // self.step_divider
        self.update_imu()
        self.update_motors()
        libslab_py.slab_update(ctypes.byref(self.slab))
        # print_ctype(self.slab.input)
        # print_ctype(self.slab.imu)
        # print_ctype(self.slab.gamepad)
        print_ctype(self.slab.state)

    def run(self):
        while True:
            p.stepSimulation()
            self.update_inputs()
            self.update_slab()
            self.steps += 1
            time.sleep(1.0 / (240.0 * p.readUserDebugParameter(self.sim_rate)))
            self.handle_reset_button()
            self.handle_center_button()
        p.disconnect()


sim = Simulation()
sim.run()
