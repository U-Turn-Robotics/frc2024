import math

import wpilib as wp
from commands2.button import CommandXboxController

import constants
from utils.utils import calcAxisSpeedWithCurvatureAndDeadzone, dz, rotate_90_degrees_ccw


class Driver:
    def __init__(self):
        # self._controller = wp.PS4Controller(constants.Pilots.k_driver_controller_port)
        self._controller = wp.XboxController(constants.Pilots.k_driver_controller_port)

    def is_connected(self):
        return self._controller.isConnected()

    def get_angle(self):
        x = self._controller.getLeftX()
        y = -self._controller.getLeftY()
        return -rotate_90_degrees_ccw(math.degrees(math.atan2(y, x)))

    def get_magnitude(self):
        x = abs(dz(self._controller.getLeftX()))
        y = abs(dz(-self._controller.getLeftY()))
        return x + y

    def get_arcade_drive_speed(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(
            self._controller.getLeftY(),
            c=2,
            b=0,
            dz=constants.Pilots.controller_deadzone,
        )

    def get_arcade_drive_rotation(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(self._controller.getRightX())

    def get_speed(self):
        # return dz(self._controller.getR2Axis() - self._controller.getL2Axis(), 0.08)
        return dz(
            self._controller.getRightTriggerAxis()
            - self._controller.getLeftTriggerAxis(),
            0.08,
        )

    def get_toggle_field_oriented(self):
        # return self._controller.getSquareButtonPressed()
        return self._controller.getXButtonPressed()

    def get_reset_angle(self):
        # return self._controller.getTriangleButtonPressed()
        return self._controller.getYButtonPressed()


class Operator:
    def __init__(self):
        # self.controller = wp.XboxController(constants.Pilots.k_operator_controller_port)
        self.controller = CommandXboxController(
            constants.Pilots.k_operator_controller_port
        )

    def get_shoot(self):
        return self.controller.a().getAsBoolean()

    def get_shoot_trigger(self):
        return self.controller.a()

    def get_shoot_speed(self):
        return (
            self.controller.getRightTriggerAxis() - self.controller.getLeftTriggerAxis()
        )
    
    def get_pickup_trigger(self):
        return self.controller.b()
