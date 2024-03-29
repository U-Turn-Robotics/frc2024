from commands2 import (
    InstantCommand,
    RunCommand,
    SequentialCommandGroup,
    StartEndCommand,
)
from cscore import CameraServer
from pathplannerlib.auto import AutoBuilder, NamedCommands, ReplanningConfig
from pathplannerlib.geometry_util import flipFieldPose
from wpilib import DriverStation, RobotBase

import constants
from AutoSelector import AutoSelector
from camera import AprilTagCamera, NoteTrackerCamera
from pilots import Driver, Operator
from subsystems.ArmSubsystem import ArmSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.PickupSubsystem import PickupSubsystem
from subsystems.ShooterSubsystem import ShooterSubsystem


class RobotContainer:
    def __init__(self) -> None:
        self.aprilTagCamera = AprilTagCamera(disabled=True)
        self.noteTrackerCamera = NoteTrackerCamera(disabled=True)

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

        self.configureAuto()
        self.startingPose = None

        self.configureButtonBindings()

        self.configureCamera()

    def teleopPeriodic(self):
        if self.operator.getGoToShootPositionPressed():
            self.armSubsystem.setShootPosition()

        if self.operator.toggleIntakeDirection():
            self.pickupSubsystem.invert()
            self.shooterSubsystem.invert()
        else:
            self.pickupSubsystem.uninvert()
            self.shooterSubsystem.uninvert()

        self.driveSubsystem.drive()

    def configureCamera(self):
        usbCam = CameraServer.startAutomaticCapture()
        # usbCam.setExposureAuto()
        usbCam.setExposureManual(35)
        usbCam.setBrightness(100)
        usbCam.setFPS(30)
        usbCam.setResolution(320, 240)

    def configureCommands(self):
        self.zeroArmPositionCommand = InstantCommand(
            self.armSubsystem.zeroPosition
        ).ignoringDisable(True)

        self.shootCommand = SequentialCommandGroup(
            StartEndCommand(
                lambda: self.armSubsystem.setSpeed(0.1),
                self.armSubsystem.atLowerLimit,
                self.armSubsystem,
            ).withTimeout(2),
            RunCommand(self.shooterSubsystem.shoot, self.shooterSubsystem).withTimeout(
                1
            ),
            RunCommand(self.pickupSubsystem.pickup, self.pickupSubsystem)
            .alongWith(
                RunCommand(
                    self.shooterSubsystem.shoot, self.shooterSubsystem
                ).withTimeout(1)
            )
            .withTimeout(1),
        ).withTimeout(4 + 0.5)

        self.pickupCommand = RunCommand(
            self.pickupSubsystem.pickup, self.pickupSubsystem
        ).andThen(InstantCommand(lambda: print("pickup tested!!!")))

    def configureButtonBindings(self):
        self.operator.getShoot().whileTrue(self.shootCommand)
        self.operator.getPickup().whileTrue(self.pickupCommand)
        # self.operator.getRaiseArm().onTrue(
        #     InstantCommand(self.armSubsystem.raisePosition, self.armSubsystem)
        # )
        # self.operator.getLowerArm().onTrue(
        #     InstantCommand(self.armSubsystem.lowerPosition, self.armSubsystem)
        # )
        self.operator.getResetArmPosition().onTrue(self.zeroArmPositionCommand)

        self.driver.getToggleFieldOriented().onTrue(
            InstantCommand(self.driveSubsystem.toggleFieldOriented).ignoringDisable(
                True
            )
        )
        self.driver.getResetAngle().onTrue(
            InstantCommand(self.driveSubsystem.resetGyro).ignoringDisable(True)
        )
        self.driver.getToggleBrakeMode().onTrue(
            InstantCommand(self.driveSubsystem.toggleBrakeMode).ignoringDisable(True)
        )
        self.driver.cycleAutoSelectorUp().onTrue(
            InstantCommand(self.autoSelector.cycleUp).ignoringDisable(True)
        )
        self.driver.cycleAutoSelectorDown().onTrue(
            InstantCommand(self.autoSelector.cycleDown).ignoringDisable(True)
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

        AutoBuilder.configureRamsete(
            self.driveSubsystem.getPose,
            self.driveSubsystem.resetPose,
            self.driveSubsystem.getSpeeds,
            self.driveSubsystem.setSpeeds,
            # (0.0625, 0.125, 2.0),  # qelems/error tolerances
            # (1.0, 2.0),  # relems/control effort
            constants.Robot.period,
            ReplanningConfig(enableDynamicReplanning=False),
            self.shouldFlipAuto,
            self.driveSubsystem,
        )

        self.autoSelector = AutoSelector()

    def shouldFlipAuto(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getAutoCommand(self):
        (self.startingPose, auto) = self.autoSelector.getSelectedAuto()
        if RobotBase.isSimulation():
            if self.startingPose:
                if self.shouldFlipAuto():
                    self.startingPose = flipFieldPose(self.startingPose)
                print(f"Starting pose: {self.startingPose}")
                self.driveSubsystem.resetPose(self.startingPose)

        initialDefaultCommand = self.driveSubsystem.getDefaultCommand()
        self.driveSubsystem.setDefaultCommand(
            # feeds the DifferentialDrive to make it stop complaining, "Output not updated often enough."
            RunCommand(self.driveSubsystem.stop, self.driveSubsystem)
        )
        if initialDefaultCommand:
            return auto.finallyDo(
                lambda: self.driveSubsystem.setDefaultCommand(initialDefaultCommand)
            )
        return auto

        return RunCommand(
            lambda: self.driveSubsystem.drivetrain.arcadeDrive(-0.25, 0, False)
        ).withTimeout(1.5)
