import math
from navx import AHRS
import wpilib as wp
from wpimath.controller import PIDController

import constants


class DriveSubsystem:
    def __init__(self, driver: wp.XboxController):
        self.driver = driver

        self.dashboard = wp.SmartDashboard

        self.gyro = AHRS(wp.SPI.Port.kMXP, update_rate_hz=100)

        self.turn_controller = PIDController(
            constants.k_turn_p, constants.k_turn_i, constants.k_turn_d, period=0.01
        )
        self.turn_controller.enableContinuousInput(-180, 180)
        self.turn_controller.setTolerance(3)

        self.field_oriented = True

    def periodic(self):
        self.driver_connected = self.driver.isConnected()
        self.dashboard.putBoolean("Driver Connected", self.driver_connected)

        self.angle = self.gyro.getYaw()
        self.dashboard.putNumber("Gyro Angle", self.angle)

    def drive(self):
        if self.driver_connected:
            if self.driver.getXButtonPressed():
                self.field_oriented = not self.field_oriented

            if self.field_oriented:
                self.field_oriented_drive()
            else:
                self.drive.arcadeDrive(self.driver.getLeftX(), self.driver.getRightX())
        else:
            self.drive.arcadeDrive(0, 0)

    def field_oriented_drive(self):
        # get the magnitude of the joystick using atan2
        x = self.driver.getLeftX()
        y = self.driver.getLeftY()

        if abs(x + y) > 0.75:
            speed = self.driver.getRightTriggerAxis() - self.driver.getLeftTriggerAxis()
            is_reversing = speed < 0

            setpoint_angle = math.degrees(math.atan2(y, x))
            if is_reversing:
                # angle = math.fmod(angle + 180, 360) - 180
                setpoint_angle = -setpoint_angle

            turn_speed = self.turn_controller.calculate(self.angle, setpoint_angle)

            # wait for the turn controller to be on target before moving
            if self.turn_controller.atSetpoint():
                self.drive.arcadeDrive(speed, turn_speed)
            else:
                self.drive.arcadeDrive(0, turn_speed)
        else:
            self.drive.arcadeDrive(0, 0)
