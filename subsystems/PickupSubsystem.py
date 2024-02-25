from commands2 import Subsystem
import rev
from constants import Arm


class PickupSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.motor = rev.CANSparkMax()
        self.motorPID = self.motor.getPIDController()
        self.motorPID.setP(Arm.k_p)
        self.motorPID.setI(Arm.k_i)
        self.motorPID.setD(Arm.k_d)

        self.lastPosition = 0

    def periodic(self):
        self.motorPID.setReference(
            self.lastPosition, rev.CANSparkMax.ControlType.kSmartMotion
        )
