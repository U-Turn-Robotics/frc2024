import rev
import wpilib as wp
import wpilib.drive
from commands2 import Subsystem
from navx import AHRS
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveOdometry,
    DifferentialDriveWheelSpeeds,
)
from wpimath.units import (
    radiansPerSecondToRotationsPerMinute,
    rotationsPerMinuteToRadiansPerSecond,
)

import constants
from pilots import Driver
from utils.utils import clamp, rotate_180_degrees


class DriveSubsystem(Subsystem):
    def __init__(self, driver: Driver):
        self.driver = driver

        # left spark 1
        spark_l_1 = rev.CANSparkMax(
            constants.Drivetrain.k_left_motor1_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        spark_l_1.restoreFactoryDefaults()
        spark_l_1.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        spark_l_1.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        spark_l_1.setInverted(True)
        self.left_encoder = spark_l_1.getEncoder()
        self.left_encoder.setPositionConversionFactor(
            constants.Drivetrain.k_position_conversion_factor
        )
        self.l_1_pid_controller = spark_l_1.getPIDController()

        # left spark 2, follows left spark 1
        spark_l_2 = rev.CANSparkMax(
            constants.Drivetrain.k_left_motor2_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        spark_l_2.restoreFactoryDefaults()
        spark_l_2.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        spark_l_2.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        spark_l_2.setInverted(True)
        self.l_2_pid_controller = spark_l_2.getPIDController()

        # right spark 1
        spark_r_1 = rev.CANSparkMax(
            constants.Drivetrain.k_right_motor1_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        spark_r_1.restoreFactoryDefaults()
        spark_r_1.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        spark_r_1.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.right_encoder = spark_r_1.getEncoder()
        self.right_encoder.setPositionConversionFactor(
            constants.Drivetrain.k_position_conversion_factor
        )
        self.r_1_pid_controller = spark_r_1.getPIDController()

        # right spark 2, follows right spark 1
        spark_r_2 = rev.CANSparkMax(
            constants.Drivetrain.k_right_motor2_port,
            rev.CANSparkMax.MotorType.kBrushless,
        )
        spark_r_2.restoreFactoryDefaults()
        spark_r_2.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        spark_r_2.setSmartCurrentLimit(constants.Drivetrain.k_dt_current_limit)
        self.r_2_pid_controller = spark_r_2.getPIDController()

        # initialize the drivetrain
        left = wpilib.MotorControllerGroup(spark_l_1, spark_l_2)
        right = wpilib.MotorControllerGroup(spark_r_1, spark_r_2)
        self.drivetrain = wpilib.drive.DifferentialDrive(left, right)

        self.gyro = AHRS(wp.SPI.Port.kMXP, update_rate_hz=100)

        self.turn_controller = PIDController(
            constants.Drivetrain.k_turn_p,
            constants.Drivetrain.k_turn_i,
            constants.Drivetrain.k_turn_d,
            period=0.01,
        )
        self.turn_controller.enableContinuousInput(-180, 180)
        self.turn_controller.setTolerance(5)

        self.field_oriented = True
        wp.SmartDashboard.putBoolean("Field Oriented", self.field_oriented)

        wp.SmartDashboard.putNumber("Setpoint angle", 0)

        self.field = wp.Field2d()
        wp.SmartDashboard.putData("Field", self.field)

        self.pose = Pose2d(angle=0, x=0, y=0)
        self.odometry = DifferentialDriveOdometry(
            self.pose.rotation(),
            self.left_encoder.getPosition(),
            self.left_encoder.getPosition(),
            initialPose=self.pose,
        )
        self.field.setRobotPose(self.pose)

    def get_angle(self):
        return self.gyro.getYaw()

    def reset_gyro(self):
        self.gyro.zeroYaw()

    def getPose(self):
        return self.pose

    def resetPose(self, pose: Pose2d):
        self.pose = pose
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.right_encoder.getPosition(),
            self.left_encoder.getPosition(),
            self.pose,
        )

    def periodic(self):
        # for calibrating encoder conversion factor
        wp.SmartDashboard.putNumber("Left Encoder", self.left_encoder.getPosition())
        wp.SmartDashboard.putNumber("Right Encoder", self.right_encoder.getPosition())

        self.pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self.right_encoder.getPosition(),
            self.left_encoder.getPosition(),
        )

        wp.SmartDashboard.putNumber("Meters X", self.pose.x)
        wp.SmartDashboard.putNumber("Meters Y", self.pose.y)

        self.field.setRobotPose(self.pose)

        self.driver_connected = self.driver.is_connected()
        wp.SmartDashboard.putBoolean("Driver Connected", self.driver_connected)

        self.angle = self.get_angle()
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

    def drive(self):
        if self.driver_connected:
            if self.driver.get_toggle_field_oriented():
                self.field_oriented = not self.field_oriented
                wp.SmartDashboard.putBoolean("Field Oriented", self.field_oriented)

            if self.driver.get_reset_angle():
                self.reset_gyro()

            if self.field_oriented:
                self.field_oriented_drive()
            else:
                self.drivetrain.arcadeDrive(
                    self.driver.get_arcade_drive_speed(),
                    self.driver.get__arcade_drive_rotation(),
                    squareInputs=False,
                )
        else:
            self.drivetrain.arcadeDrive(0, 0)

    def field_oriented_drive(self):
        speed = self.driver.get_speed()
        turn_speed = 0.0

        # get the magnitude of the joystick using atan2
        if self.driver.get_magnitude() > 0.9 and abs(speed) > 0.0:
            setpoint_angle = self.driver.get_angle()

            is_reversing = speed < 0
            if is_reversing:
                setpoint_angle = rotate_180_degrees(setpoint_angle)

            wp.SmartDashboard.putNumber("Setpoint angle", setpoint_angle)
            self.turn_controller.setSetpoint(setpoint_angle)
            turn_speed = self.turn_controller.calculate(self.angle)
            turn_speed = clamp(
                -constants.Drivetrain.k_turn_max_speed,
                turn_speed,
                constants.Drivetrain.k_turn_max_speed,
            )

            # # wait for the turn controller to be on target before moving
            # if not self.turn_controller.atSetpoint():
            #     speed = 0
        else:
            speed = 0

        self.drivetrain.arcadeDrive(
            -speed * constants.Drivetrain.speed_scale,
            turn_speed,
            squareInputs=True,
        )
