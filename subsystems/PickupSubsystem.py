import rev
from commands2 import Subsystem
from wpilib import RobotBase

import constants


class PickupSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.motor = rev.CANSparkMax(
            constants.Pickup.k_motor_id,
            rev.CANSparkMax.MotorType.kBrushed
            if not RobotBase.isSimulation()
            else rev.CANSparkMax.MotorType.kBrushless,
        )
        self.motor.setSmartCurrentLimit(60)

        self._invert = 1

    def invert(self):
        self._invert = -1

    def uninvert(self):
        self._invert = 1

    def pickup(self):
        self.motor.set(constants.Pickup.k_pickup_speed * self._invert)

    def stop(self):
        self.motor.set(0)

    def setSpeed(self, speed: float):
        self.motor.set(speed)
