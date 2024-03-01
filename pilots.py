import math

from commands2.button import CommandXboxController

import constants
from utils.utils import calcAxisSpeedWithCurvatureAndDeadzone, dz, rotate_90_degrees_ccw


class Driver:
    def __init__(self):
        self._controller = CommandXboxController(
            constants.Pilots.k_driver_controller_port
        )

    def isConnected(self):
        return self._controller._hid.isConnected()

    def getAngle(self):
        x = self._controller.getLeftX()
        y = -self._controller.getLeftY()
        return -rotate_90_degrees_ccw(math.degrees(math.atan2(y, x)))

    def getMagnitude(self):
        x = abs(dz(self._controller.getLeftX()))
        y = abs(dz(-self._controller.getLeftY()))
        return x + y

    def getArcadeDriveSpeed(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(
            self._controller.getLeftY(),
            c=2,
            b=0,
            dz=constants.Pilots.controller_deadzone,
        )

    def getArcadeDriveRotation(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(self._controller.getRightX())

    def getSpeed(self):
        return dz(
            self._controller.getRightTriggerAxis()
            - self._controller.getLeftTriggerAxis(),
            0.08,
        )

    def getToggleFieldOriented(self):
        return self._controller.x()

    def getResetAngle(self):
        return self._controller.y()

    def getToggleBrakeMode(self):
        return self._controller.start()

    def getTrackNoteGamePiece(self):
        return self._controller._hid.getAButton()

    def getSnapToClosestPresetPose(self):
        return self._controller._hid.getBButton()


class Operator:
    def __init__(self):
        self._controller = CommandXboxController(
            constants.Pilots.k_operator_controller_port
        )

    def isConnected(self):
        return self._controller._hid.isConnected()

    def getShoot(self):
        return self._controller.a()

    def getShootSpeed(self):
        return self._controller.getLeftTriggerAxis()

    def getPickup(self):
        return self._controller.b()

    def getPickupSpeed(self):
        return self._controller.getRightTriggerAxis()

    def toggleIntakeDirection(self):
        return self._controller.y()

    def getRaiseArm(self):
        return self._controller.povUp()

    def getLowerArm(self):
        return self._controller.povDown()

    def getArmSpeed(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(self._controller.getLeftY(), 2)

    def getSlowArmSpeed(self):
        return calcAxisSpeedWithCurvatureAndDeadzone(self._controller.getRightY(), 2)
