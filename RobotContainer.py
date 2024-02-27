from commands2 import InstantCommand, RunCommand, StartEndCommand
from pathplannerlib.auto import AutoBuilder, NamedCommands, ReplanningConfig
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation

import constants
from AutoSelector import AutoSelector
from camera import AprilTagCamera, NoteTrackerCamera
from pilots import Driver, Operator
from subsystems.ArmSubsystem import ArmSubsystem
from subsystems.ConveyorSubsystem import ConveyorSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.PickupSubsystem import PickupSubsystem
from subsystems.ShooterSubsystem import ShooterSubsystem


class RobotContainer:
    def __init__(self) -> None:
        self.aprilTagCamera = AprilTagCamera()
        self.noteTrackerCamera = NoteTrackerCamera()

        self.driver = Driver()
        self.driveSubsystem = DriveSubsystem(
            self.driver, self.aprilTagCamera, self.noteTrackerCamera
        )
        self.driveSubsystem.setDefaultCommand(
            RunCommand(self.driveSubsystem.drive, self.driveSubsystem)
        )

        self.operator = Operator()

        self.pickupSubsystem = PickupSubsystem()
        self.pickupSubsystem.setDefaultCommand(
            RunCommand(self.pickupSubsystem.stop, self.pickupSubsystem)
        )
        self.conveyorSubsystem = ConveyorSubsystem()
        self.conveyorSubsystem.setDefaultCommand(
            RunCommand(self.conveyorSubsystem.stop, self.conveyorSubsystem)
        )
        self.shooterSubsystem = ShooterSubsystem()
        self.shooterSubsystem.setDefaultCommand(
            RunCommand(self.shooterSubsystem.stop, self.shooterSubsystem)
        )
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
        self.operator.getShoot().whileTrue(self.shootCommand)
        self.operator.getPickup().whileTrue(self.pickupCommand)

        self.driver.getToggleFieldOriented().onTrue(
            InstantCommand(self.driveSubsystem.toggleFieldOriented)
        )
        self.driver.getResetAngle().onTrue(
            InstantCommand(self.driveSubsystem.resetGyro)
        )
        self.driver.getToggleBrakeMode().onTrue(
            InstantCommand(self.driveSubsystem.toggleBrakeMode)
        )

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
