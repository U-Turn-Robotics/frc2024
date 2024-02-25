import rev
from commands2 import Subsystem

import constants


class PickupSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.motor = rev.CANSparkMax(constants.Pickup.k_motor_id)

    def pickup(self):
        self.motor.set(constants.Pickup.k_pickup_speed)

    def stop(self):
        self.motor.set(0)
