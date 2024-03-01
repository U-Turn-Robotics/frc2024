import math

import rev
import wpilib as wp
import wpilib.drive
from commands2 import Subsystem
from navx import AHRS
from pathplannerlib.auto import AutoBuilder, PathConstraints
from pathplannerlib.path import GoalEndState, PathPlannerPath
from wpimath.controller import PIDController
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from wpimath.units import (
    radiansPerSecondToRotationsPerMinute,
    rotationsPerMinuteToRadiansPerSecond,
)

import constants
from camera import AprilTagCamera, NoteTrackerCamera
from pilots import Driver
from utils.utils import clamp, findClosestPose, rotate_180_degrees


class DriveSubsystem(Subsystem):
    def __init__(
        self,
        driver: Driver,
        aprilTagCamera: AprilTagCamera,
        noteTrackerCamera: NoteTrackerCamera,
    ):
        self.driver = driver
        self.aprilTagCamera = aprilTagCamera
        self.noteTrackerCamera = noteTrackerCamera

        # left spark 1
        self.spark_l_1 = rev.CANSparkMax(
            constants.Drivetrain.k_left_motor1_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        self.spark_l_1.restoreFactoryDefaults()
        self.spark_l_1.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.spark_l_1.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.spark_l_1.setInverted(True)
        self.left_encoder = self.spark_l_1.getEncoder()
        self.left_encoder.setPositionConversionFactor(
            constants.Drivetrain.k_position_conversion_factor
        )
        self.l_1_pid_controller = self.spark_l_1.getPIDController()

        # left spark 2, follows left spark 1
        self.spark_l_2 = rev.CANSparkMax(
            constants.Drivetrain.k_left_motor2_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        self.spark_l_2.restoreFactoryDefaults()
        self.spark_l_2.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.spark_l_2.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.spark_l_2.setInverted(True)
        self.l_2_pid_controller = self.spark_l_2.getPIDController()

        # right spark 1
        self.spark_r_1 = rev.CANSparkMax(
            constants.Drivetrain.k_right_motor1_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        self.spark_r_1.restoreFactoryDefaults()
        self.spark_r_1.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.spark_r_1.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.right_encoder = self.spark_r_1.getEncoder()
        self.right_encoder.setPositionConversionFactor(
            constants.Drivetrain.k_position_conversion_factor
        )
        self.r_1_pid_controller = self.spark_r_1.getPIDController()

        # right spark 2, follows right spark 1
        self.spark_r_2 = rev.CANSparkMax(
            constants.Drivetrain.k_right_motor2_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        self.spark_r_2.restoreFactoryDefaults()
        self.spark_r_2.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.spark_r_2.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.r_2_pid_controller = self.spark_r_2.getPIDController()

        # initialize the drivetrain
        left = wpilib.MotorControllerGroup(self.spark_l_1, self.spark_l_2)
        right = wpilib.MotorControllerGroup(self.spark_r_1, self.spark_r_2)
        self.drivetrain = wpilib.drive.DifferentialDrive(left, right)

        self.gyro = AHRS(wp.SPI.Port.kMXP, update_rate_hz=100)

        self.turnController = PIDController(
            constants.Drivetrain.k_turn_p,
            constants.Drivetrain.k_turn_i,
            constants.Drivetrain.k_turn_d,
            period=constants.Robot.period,
        )
        self.turnController.enableContinuousInput(-180, 180)
        self.turnController.setTolerance(5)

        self.vision_forward_controller = PIDController(
            constants.Drivetrain.k_forward_p,
            constants.Drivetrain.k_forward_i,
            constants.Drivetrain.k_forward_d,
            period=constants.Robot.period,
        )
        self.vision_turn_controller = PIDController(
            constants.Drivetrain.k_turn_p,
            constants.Drivetrain.k_turn_i,
            constants.Drivetrain.k_turn_d,
            period=constants.Robot.period,
        )

        self.field_oriented = True
        wp.SmartDashboard.putBoolean("Field Oriented", self.field_oriented)

        wp.SmartDashboard.putNumber("Setpoint angle", 0)

        self.field = wp.Field2d()
        wp.SmartDashboard.putData("Field", self.field)

        self.pose = Pose2d(angle=0, x=0, y=0)
        self.poseEstimator = DifferentialDrivePoseEstimator(
            constants.Drivetrain.differential_drive_kinematics,
            self.pose.rotation(),
            self.left_encoder.getPosition(),
            self.left_encoder.getPosition(),
            self.pose,
            # TODO tune these, these are the default values
            (0.05, 0.05, math.radians(5)),
            (0.5, 0.5, math.radians(30)),
        )
        self.field.setRobotPose(self.pose)

        self.pathfindCommand = None

    def getAngle(self):
        return self.gyro.getYaw()

    def resetGyro(self):
        self.gyro.zeroYaw()

    def getPose(self):
        return self.pose

    def resetPose(self, pose: Pose2d):
        self.pose = pose
        self.poseEstimator.resetPosition(
            self.gyro.getRotation2d(),
            self.right_encoder.getPosition(),
            self.left_encoder.getPosition(),
            self.pose,
        )

    def periodic(self):
        # for calibrating encoder conversion factor
        wp.SmartDashboard.putNumber("Left Encoder", self.left_encoder.getPosition())
        wp.SmartDashboard.putNumber("Right Encoder", self.right_encoder.getPosition())

        self.pose = self.poseEstimator.update(
            self.gyro.getRotation2d(),
            self.right_encoder.getPosition(),
            self.left_encoder.getPosition(),
        )

        (estimatedPose, timestamp) = self.aprilTagCamera.getEstimatedPose(
            self.pose, self.getSpeeds()
        )
        if estimatedPose is not None:
            self.poseEstimator.addVisionMeasurement(estimatedPose, timestamp)

        wp.SmartDashboard.putNumber("Meters X", self.pose.x)
        wp.SmartDashboard.putNumber("Meters Y", self.pose.y)

        self.field.setRobotPose(self.pose)

        self.driver_connected = self.driver.isConnected()
        wp.SmartDashboard.putBoolean("Driver Connected", self.driver_connected)

        self.angle = self.getAngle()
        wp.SmartDashboard.putNumber("Gyro Angle", self.angle)

    def getSpeeds(self):
        leftRPM = self.left_encoder.getVelocity()
        rightRPM = self.right_encoder.getVelocity()

        leftAngularVel = rotationsPerMinuteToRadiansPerSecond(leftRPM)
        rightAngularVel = rotationsPerMinuteToRadiansPerSecond(rightRPM)

        leftLinearVel = leftAngularVel * constants.Drivetrain.kWheelRadiusMeters
        rightLinearVel = rightAngularVel * constants.Drivetrain.kWheelRadiusMeters

        return constants.Drivetrain.differential_drive_kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(leftLinearVel, rightLinearVel)
        )

    def setSpeeds(self, speeds: ChassisSpeeds):
        wheelSpeeds = constants.Drivetrain.differential_drive_kinematics.toWheelSpeeds(
            speeds
        )
        wheelSpeeds.desaturate(constants.Drivetrain.k_max_velocity)

        leftAngularVel = wheelSpeeds.left / constants.Drivetrain.kWheelRadiusMeters
        rightAngularVel = wheelSpeeds.right / constants.Drivetrain.kWheelRadiusMeters

        leftRPM = radiansPerSecondToRotationsPerMinute(leftAngularVel)
        rightRPM = radiansPerSecondToRotationsPerMinute(rightAngularVel)

        self.l_1_pid_controller.setReference(
            leftRPM, rev.CANSparkMax.ControlType.kVelocity
        )
        self.l_2_pid_controller.setReference(
            leftRPM, rev.CANSparkMax.ControlType.kVelocity
        )
        self.r_1_pid_controller.setReference(
            rightRPM, rev.CANSparkMax.ControlType.kVelocity
        )
        self.r_2_pid_controller.setReference(
            rightRPM, rev.CANSparkMax.ControlType.kVelocity
        )

        # prevents "Error at frc::MotorSafety::Check: A timeout has been exceeded: DifferentialDrive... Output not updated often enough."
        self.drivetrain.feed()

    def toggleBrakeMode(self):
        idleModeToSet = (
            rev.CANSparkMax.IdleMode.kBrake
            if self.spark_l_1.getIdleMode() == rev.CANSparkMax.IdleMode.kCoast
            else rev.CANSparkMax.IdleMode.kCoast
        )
        self.spark_l_1.setIdleMode(idleModeToSet)
        self.spark_l_2.setIdleMode(idleModeToSet)
        self.spark_r_1.setIdleMode(idleModeToSet)
        self.spark_r_2.setIdleMode(idleModeToSet)

    def toggleFieldOriented(self):
        self.field_oriented = not self.field_oriented
        wp.SmartDashboard.putBoolean("Field Oriented", self.field_oriented)

    def drive(self):
        speed = 0.0
        turnSpeed = 0.0
        squareInputs = True

        isVisionTracking = False

        if self.driver_connected:
            if self.field_oriented:
                (speed, turnSpeed) = self.fieldOrientedDrive()
            else:
                (speed, turnSpeed) = self.arcadeDrive()

            if speed == 0.0:
                if self.driver.getTrackNoteGamePiece():
                    (speed, turnSpeed) = self.visionTrackNote()
                    isVisionTracking = True
                else:
                    if self.driver.getSnapToClosestPresetPose():
                        self.snapToClosestPresetPose()
                        # skip the arcade drive since pathfinding handles the driving
                        return
                    else:
                        self.resetPathfindCommand()

        if not isVisionTracking:
            self.noteTrackerCamera.disableTracking()

        if not wp.RobotBase.isSimulation():
            self.drivetrain.arcadeDrive(speed, turnSpeed, squareInputs)
        else:
            wheelSpeeds = self.drivetrain.arcadeDriveIK(speed, turnSpeed, squareInputs)
            self.setSpeeds(
                constants.Drivetrain.differential_drive_kinematics.toChassisSpeeds(
                    DifferentialDriveWheelSpeeds(wheelSpeeds.left, wheelSpeeds.right)
                )
            )

    def stop(self):
        self.drivetrain.stopMotor()

    def arcadeDrive(self):
        speed = self.driver.getArcadeDriveSpeed()
        turnSpeed = self.driver.getArcadeDriveRotation()
        if abs(turnSpeed) > 0:
            self.lastTurnAngle = self.angle
        return (speed, turnSpeed)

    def fieldOrientedDrive(self):
        speed = self.driver.getSpeed()
        turnSpeed = 0.0

        if self.driver.getMagnitude() > 0.8 and abs(speed) > 0.05:
            setpointAngle = self.driver.getAngle()

            is_reversing = speed < 0.0
            if is_reversing:
                setpointAngle = rotate_180_degrees(setpointAngle)

            turnSpeed = self._getTurnPIDSpeed(setpointAngle)

            # # wait for the turn controller to be on target before moving
            # if not self.turn_controller.atSetpoint():
            #     speed = 0

        return (-speed * constants.Drivetrain.speed_scale, turnSpeed)

    def visionTrackNote(self):
        self.noteTrackerCamera.enableTracking()

        speed = 0.0
        turnSpeed = 0.0
        (angle, area) = self.noteTrackerCamera.getNotePosition()
        if angle and area:
            turnSpeed = -self.vision_turn_controller.calculate(angle, 0)
            speed = -self.vision_forward_controller.calculate(area, 100)

        return (speed, turnSpeed)

    def snapToClosestPresetPose(self):
        if self.pathfindCommand is None:
            currentPose = self.getPose()
            closestPose = findClosestPose(
                currentPose,
                constants.Robot.preset_positions_list_flipped
                if AutoBuilder._shouldFlipPath()
                else constants.Robot.preset_positions_list,
            )

            bezierPoints = PathPlannerPath.bezierFromPoses(
                [
                    currentPose,
                    closestPose,
                ]
            )
            constraints = PathConstraints(
                constants.Drivetrain.k_max_velocity,
                constants.Drivetrain.k_max_acceleration,
                constants.Drivetrain.k_max_turn_velocity,
                constants.Drivetrain.k_max_turn_acceleration,
            )
            path = PathPlannerPath(
                bezierPoints,
                constraints,
                GoalEndState(0.0, closestPose.rotation(), True),
            )
            # the preset poses are already flipped, so the path should not be flipped
            path.preventFlipping = True

            self.pathfindCommand = AutoBuilder.followPath(path)

            self.pathfindCommand.initialize()

        self.pathfindCommand.execute()
        if self.pathfindCommand.isFinished():
            self.pathfindCommand.end(False)
            self.stop()

    def resetPathfindCommand(self):
        if self.pathfindCommand:
            self.pathfindCommand = None

    def _getTurnPIDSpeed(self, setpointAngle: float):
        wp.SmartDashboard.putNumber("Setpoint angle", setpointAngle)
        self.turnController.setSetpoint(setpointAngle)
        turnSpeed = self.turnController.calculate(self.angle)
        turnSpeed = clamp(
            turnSpeed,
            -constants.Drivetrain.k_turn_max_speed,
            constants.Drivetrain.k_turn_max_speed,
        )
        return turnSpeed
