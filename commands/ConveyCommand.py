from commands2 import Command
from subsystems.ConveyorSubsystem import ConveyorSubsystem


class ConveyCommand(Command):
    def __init__(self, conveyorSubsystem: ConveyorSubsystem):
        super().__init__()

        self.conveyorSubsystem = conveyorSubsystem
        self.addRequirements(self.conveyorSubsystem)

    def execute(self):
        self.conveyorSubsystem.convey()

    def end(self):
        self.conveyorSubsystem.stop()
