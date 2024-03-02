from commands2 import InstantCommand, RunCommand, StartEndCommand
from pathplannerlib.auto import AutoBuilder, NamedCommands, ReplanningConfig
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation, RobotBase

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

        self.operator = Operator()

        self.pickupSubsystem = PickupSubsystem()
        self.pickupSubsystem.setDefaultCommand(
            RunCommand(
                lambda: self.pickupSubsystem.setSpeed(self.operator.getPickupSpeed()),
                self.pickupSubsystem,
            )
        )
        self.shooterSubsystem = ShooterSubsystem()
        self.shooterSubsystem.setDefaultCommand(
            RunCommand(
                lambda: self.shooterSubsystem.setSpeed(self.operator.getShootSpeed()),
                self.shooterSubsystem,
            )
        )
        self.armSubsystem = ArmSubsystem()
        self.armSubsystem.setDefaultCommand(
            RunCommand(
                lambda: self.armSubsystem.setSpeed(
                    self.operator.getSlowArmSpeed() * constants.Arm.slow_arm_speed_scale
                    or self.operator.getArmSpeed() * constants.Arm.arm_speed_scale
                ),
                self.armSubsystem,
            )
        )

        self.configureCommands()

        self.configureButtonBindings()

        self.configureAuto()
        self.startingPose = None

    def teleopPeriodic(self):
        self.driveSubsystem.drive()

    def configureCommands(self):
        self.shootCommand = (
            # RunCommand(lambda: self.armSubsystem.setSpeed(-0.2), self.armSubsystem)
            StartEndCommand(
                lambda: self.armSubsystem.setSpeed(-0.1),
                lambda: not self.armSubsystem.limitSwitch.get(),
                self.armSubsystem,
            )
            .withTimeout(2)
            .andThen(RunCommand(self.shooterSubsystem.shoot, self.shooterSubsystem))
            .withTimeout(0.5)
            .andThen(
                RunCommand(self.pickupSubsystem.pickup, self.pickupSubsystem).alongWith(
                    RunCommand(self.shooterSubsystem.shoot, self.shooterSubsystem)
                )
            )
            .withTimeout(1)
            .andThen(InstantCommand(lambda: print("shoot tested!!!")))
        )

        self.pickupCommand = RunCommand(
            self.pickupSubsystem.pickup, self.pickupSubsystem
        ).andThen(InstantCommand(lambda: print("pickup tested!!!")))

    def configureButtonBindings(self):
        self.operator.getShoot().whileTrue(self.shootCommand)
        self.operator.getPickup().whileTrue(self.pickupCommand)
        self.operator.getRaiseArm().onTrue(
            InstantCommand(self.armSubsystem.raisePosition, self.armSubsystem)
        )
        self.operator.getLowerArm().onTrue(
            InstantCommand(self.armSubsystem.lowerPosition, self.armSubsystem)
        )

        def invertBoth():
            self.pickupSubsystem.invert()
            self.shooterSubsystem.invert()

        def uninvertBoth():
            self.pickupSubsystem.uninvert()
            self.shooterSubsystem.uninvert()

        self.operator.toggleIntakeDirection().whileTrue(
            StartEndCommand(
                invertBoth, uninvertBoth, self.pickupSubsystem, self.shooterSubsystem
            )
        )

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
            self.shootCommand,
        )
        NamedCommands.registerCommand(
            "pickup",
            self.pickupCommand,
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
        # (self.startingPose, auto) = self.autoSelector.getSelectedAuto()
        # if RobotBase.isSimulation():
        #     if self.startingPose:
        #         if self.shouldFlipAuto():
        #             self.startingPose = flipFieldPose(self.startingPose)
        #         print(f"Starting pose: {self.startingPose}")
        #         self.driveSubsystem.resetPose(self.startingPose)
        # return auto
        return RunCommand(
            lambda: self.driveSubsystem.drivetrain.arcadeDrive(0.5, 0, False)
        ).withTimeout(1)
