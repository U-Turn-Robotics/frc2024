from commands2 import Command
from subsystems.ShooterSubsystem import ShooterSubsystem
import wpilib as wp


class ShootCommand(Command):
    def __init__(self, shooterSubsystem: ShooterSubsystem):
        super().__init__()

        self.shooterSubsystem = shooterSubsystem
        self.addRequirements(self.shooterSubsystem)
        self.timer = wp.Timer()

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def isFinished(self):
        self.timer.get() > 1

    def execute(self):
        self.shooterSubsystem.shoot()

    def end(self):
        self.shooterSubsystem.stop()
