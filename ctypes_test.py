#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import numpy as np
import time
import inputs, os, io, fcntl
import json

import ctypes
from controller.build import slab_ctypes

libslab = slab_ctypes._libs["libslab_controller.so"]


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
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        # add user interface
        self.reset_button_count = 0
        self.reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
        self.center_button_count = 0
        self.center_button = p.addUserDebugParameter("center", 1, 0, 0)
        self.sim_rate = p.addUserDebugParameter("sim_rate", 0, 5, 1)
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
        self.joints = [
            p.getJointInfo(self.robot, i) for i in range(p.getNumJoints(self.robot))
        ]
        self.legs = [joint[0] for joint in self.joints if joint[1].endswith(b"leg")]
        self.wheels = [joint[0] for joint in self.joints if joint[1].endswith(b"wheel")]
        # start with legs folded
        for leg in self.legs:
            p.resetJointState(self.robot, leg, -np.pi, 0)
        # reset camera
        p.resetDebugVisualizerCamera(5, 50, -35, (0, 0, 0))
        self.slab = slab_ctypes.Slab()
        self.slab.config.max_wheel_speed = 40  # rad/s
        self.slab.config.wheel_diameter = 0.165  # m
        self.slab.config.wheel_distance = 0.4  # m

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
        # TODO set motor feedback
        p.setJointMotorControlArray(
            self.robot,
            self.wheels,
            p.VELOCITY_CONTROL,
            targetVelocities=[
                self.slab.motors[slab_ctypes.MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity,
                -self.slab.motors[slab_ctypes.MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity,
                self.slab.motors[slab_ctypes.MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity,
                -self.slab.motors[slab_ctypes.MOTOR_ID_BACK_LEFT_WHEEL].input.velocity,
            ],
        )

        p.setJointMotorControlArray(
            self.robot,
            self.legs,
            p.POSITION_CONTROL,
            targetPositions=[
                self.slab.motors[slab_ctypes.MOTOR_ID_FRONT_LEGS].input.position,
                self.slab.motors[slab_ctypes.MOTOR_ID_BACK_LEGS].input.position,
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
        # print(self.inputs)
        dpad = np.zeros(2)
        dpad[0] += self.inputs.get("BTN_DPAD_UP", 0)
        dpad[0] -= self.inputs.get("BTN_DPAD_DOWN", 0)
        dpad[1] += self.inputs.get("BTN_DPAD_LEFT", 0)
        dpad[1] -= self.inputs.get("BTN_DPAD_RIGHT", 0)
        if np.any(dpad):
            dpad *= self.inputs.get("ABS_RZ", 0) / (255 * np.linalg.norm(dpad))
            dpad *= (
                self.slab.config.max_wheel_speed * 0.5 * self.slab.config.wheel_diameter
            )
        self.slab.input.linear_velocity = dpad[0]
        self.slab.input.angular_velocity = dpad[1] / (
            0.5 * self.slab.config.wheel_distance
        )

    def update_slab(self):
        if self.steps % self.step_divider > 0:
            return
        self.slab.tick = self.steps // self.step_divider
        self.update_imu()
        self.update_motors()
        libslab.slab_update(ctypes.byref(self.slab))
        # print_ctype(self.slab.imu)
        # print_ctype(self.slab.input)

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
