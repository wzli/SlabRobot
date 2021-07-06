#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import numpy as np
import time

import ctypes
from controller.build import slab_ctypes

libslab = slab_ctypes._libs["libslab_controller.so"]


class Simulation:
    def __init__(self):
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

    def handle_center_button(self):
        if p.readUserDebugParameter(self.center_button) > self.center_button_count:
            self.center_button_count += 1
            camera_info = p.getDebugVisualizerCamera()
            pos, orn = p.getBasePositionAndOrientation(self.robot)
            p.resetDebugVisualizerCamera(
                camera_info[10], camera_info[8], camera_info[9], pos
            )

    def handle_reset_button(self):
        if p.readUserDebugParameter(self.reset_button) > self.reset_button_count:
            self.reset_button_count += 1
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

    def update_slab(self):
        if self.steps % self.step_divider > 0:
            return
        self.slab.tick = self.steps // self.step_divider
        self.update_imu()
        self.update_motors()
        libslab.slab_update(ctypes.byref(self.slab))

    def run(self):
        while True:
            p.stepSimulation()
            self.update_slab()
            self.steps += 1
            time.sleep(1.0 / (240.0 * p.readUserDebugParameter(self.sim_rate)))
            self.handle_reset_button()
            self.handle_center_button()
        p.disconnect()


sim = Simulation()
sim.run()
