from commands2 import InstantCommand, RunCommand, StartEndCommand
from pathplannerlib.auto import AutoBuilder, NamedCommands, ReplanningConfig
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation

import constants
from AutoSelector import AutoSelector
from camera import AprilTagCamera
from pilots import Driver, Operator
from subsystems.ArmSubsystem import ArmSubsystem
from subsystems.ConveyorSubsystem import ConveyorSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.PickupSubsystem import PickupSubsystem
from subsystems.ShooterSubsystem import ShooterSubsystem


class RobotContainer:
    def __init__(self) -> None:
        self.aprilTagCamera = AprilTagCamera()

        self.driver = Driver()
        self.driveSubsystem = DriveSubsystem(self.driver, self.aprilTagCamera)
        self.driveSubsystem.setDefaultCommand(
            RunCommand(self.driveSubsystem.drive, self.driveSubsystem)
        )

        self.operator = Operator()

        self.pickupSubsystem = PickupSubsystem()
        self.pickupSubsystem.setDefaultCommand(self.pickupSubsystem.stop)
        self.conveyorSubsystem = ConveyorSubsystem()
        self.conveyorSubsystem.setDefaultCommand(self.conveyorSubsystem.stop)
        self.shooterSubsystem = ShooterSubsystem()
        self.shooterSubsystem.setDefaultCommand(self.shooterSubsystem.stop)
        self.armSubsystem = ArmSubsystem()

        self.configureCommands()

        self.configureButtonBindings()

        self.configureAuto()
        self.startingPose = None

    def configureCommands(self):
        self.shootCommand = (
            RunCommand(self.shooterSubsystem.shoot, self.shooterSubsystem)
            .withTimeout(0.5)
            .andThen(
                StartEndCommand(
                    self.conveyorSubsystem.convey,
                    self.conveyorSubsystem.stop,
                    self.conveyorSubsystem,
                )
            )
        )

        self.pickupCommand = RunCommand(
            self.pickupSubsystem.pickup, self.pickupSubsystem
        )

    def configureButtonBindings(self):
        self.operator.get_shoot_trigger().whileTrue(self.shootCommand)
        self.operator.get_pickup_trigger().whileTrue(self.pickupCommand)

    def configureAuto(self):
        NamedCommands.registerCommand(
            "shoot",
            self.shootCommand.andThen(InstantCommand(lambda: print("shoot tested!!!"))),
        )
        NamedCommands.registerCommand(
            "pickup",
            self.pickupCommand.andThen(
                InstantCommand(lambda: print("pickup tested!!!"))
            ),
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
