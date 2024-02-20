from commands2 import InstantCommand, RunCommand
from pathplannerlib.auto import AutoBuilder, NamedCommands, ReplanningConfig
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation

import constants
from AutoSelector import AutoSelector
from pilots import Driver, Operator
from subsystems.DriveSubsystem import DriveSubsystem


class RobotContainer:
    def __init__(self) -> None:
        self.driver = Driver()
        self.driveSubsystem = DriveSubsystem(self.driver)
        self.driveSubsystem.setDefaultCommand(
            RunCommand(self.driveSubsystem.drive, self.driveSubsystem)
        )

        self.operator = Operator()

        self.configureButtonBindings()

        self.configureAuto()
        self.startingPose = None

    def configureButtonBindings(self):
        pass

    def configureAuto(self):
        NamedCommands.registerCommand(
            "shoot", InstantCommand(lambda: print("shoot tested!!!"))
        )
        NamedCommands.registerCommand(
            "pickup", InstantCommand(lambda: print("pickup tested!!!"))
        )

        AutoBuilder.configureLTV(
            self.driveSubsystem.getPose,
            self.driveSubsystem.resetPose,
            self.driveSubsystem.getSpeeds,
            self.driveSubsystem.setSpeeds,
            (0.0625, 0.125, 2.0),  # qelems/error tolerances
            (1.0, 2.0),  # relems/control effort
            constants.Robot.period,
            ReplanningConfig(enableDynamicReplanning=True),
            self.shouldFlipAuto,
            self.driveSubsystem,
        )

        self.autoSelector = AutoSelector()

    def shouldFlipAuto(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getAutoCommand(self):
        (self.startingPose, auto) = self.autoSelector.getSelectedAuto()
        print(f"Starting pose: {self.startingPose}")
        if self.startingPose:
            if self.shouldFlipAuto():
                self.startingPose = flipFieldPose(self.startingPose)
            self.driveSubsystem.resetPose(self.startingPose)
        return auto
