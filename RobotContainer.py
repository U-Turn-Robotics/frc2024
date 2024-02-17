from commands2 import InstantCommand, RunCommand
from pathplannerlib.auto import (
    AutoBuilder,
    NamedCommands,
    PathPlannerAuto,
    ReplanningConfig,
)
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation

import constants
from pilots import Driver
from subsystems.DriveSubsystem import DriveSubsystem


class RobotContainer:
    def __init__(self) -> None:

        self.driver = Driver()
        self.driveSubsystem = DriveSubsystem(self.driver)
        self.driveSubsystem.setDefaultCommand(
            RunCommand(self.driveSubsystem.drive, self.driveSubsystem)
        )

        # self.operator = wp.XboxController(1)

        self.configureButtonBindings()

        self.configureAuto()

        self.auto = self.loadAuto()

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

    def shouldFlipAuto(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def loadAuto(self):
        return PathPlannerAuto("Auto-1")

    def loadAutoStartPose(self):
        start_pose = PathPlannerAuto.getStartingPoseFromAutoFile("Auto-1")
        return flipFieldPose(start_pose) if self.shouldFlipAuto() else start_pose

    def getAutoCommand(self):
        return self.auto
