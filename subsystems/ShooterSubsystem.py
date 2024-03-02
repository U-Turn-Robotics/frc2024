import rev
import wpilib as wp
from commands2 import Subsystem

import constants


class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        motor_1 = rev.CANSparkFlex(
            constants.Shooter.k_motor_1_id, rev.CANSparkFlex.MotorType.kBrushless
        )
        motor_1.restoreFactoryDefaults()
        motor_1.setSmartCurrentLimit(60)
        motor_1.setIdleMode(rev.CANSparkFlex.IdleMode.kCoast)

        motor_2 = rev.CANSparkFlex(
            constants.Shooter.k_motor_2_id, rev.CANSparkFlex.MotorType.kBrushless
        )
        motor_2.restoreFactoryDefaults()
        motor_2.setSmartCurrentLimit(60)
        motor_2.setIdleMode(rev.CANSparkFlex.IdleMode.kCoast)

        self.shooter = wp.MotorControllerGroup(motor_1, motor_2)

        self._invert = 1

    def invert(self):
        self._invert = -1

    def uninvert(self):
        self._invert = 1

    def setSpeed(self, speed: float):
        self.shooter.set(speed * self._invert)

    def shoot(self):
        self.shooter.set(constants.Shooter.k_shoot_speed)

    def stop(self):
        self.shooter.set(0)
