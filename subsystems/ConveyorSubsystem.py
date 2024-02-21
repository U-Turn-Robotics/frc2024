from commands2 import Subsystem
import wpilib as wp


class ConveyorSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.conveyor = wp.Servo(1)

    def convey(self):
        self.conveyor.set(1)

    def stop(self):
        self.conveyor.set(0.5)
