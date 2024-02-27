import rev
from commands2 import Subsystem
from wpimath.controller import ArmFeedforward
from wpimath.units import rotationsPerMinuteToRadiansPerSecond

from constants import Arm


class ArmSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.motor = rev.CANSparkMax(
            Arm.k_motor_id, rev.CANSparkMax.MotorType.kBrushless
        )
        self.motor.restoreFactoryDefaults()
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.motor.setOpenLoopRampRate(0.5)

        self.motorEncoder = self.motor.getEncoder()
        self.motorEncoder.setPositionConversionFactor(Arm.k_encoder_position_per_radian)
        self.motorEncoder.setVelocityConversionFactor(
            rotationsPerMinuteToRadiansPerSecond(Arm.k_encoder_position_per_radian)
        )

        self.motorPID = self.motor.getPIDController()
        self.motorPID.setP(Arm.k_p)
        self.motorPID.setI(Arm.k_i)
        self.motorPID.setD(Arm.k_d)
        self.motorPID.setSmartMotionAccelStrategy(
            rev.SparkMaxPIDController.AccelStrategy.kSCurve
        )
        self.motorPID.setSmartMotionMaxVelocity(Arm.k_max_velocity)
        self.motorPID.setSmartMotionMaxAccel(Arm.k_max_acceleration)
        self.motorPID.setSmartMotionAllowedClosedLoopError(0)

        self.armFF = ArmFeedforward(Arm.k_s, Arm.k_g, Arm.k_v, Arm.k_a)

        self.lastPosition = 0

    def periodic(self):
        motorPos = self.motorEncoder.getPosition()
        motorVel = self.motorEncoder.getVelocity()
        armFFVoltage = self.armFF.calculate(motorPos, motorVel)

        self.motorPID.setReference(
            self.lastPosition,
            rev.CANSparkMax.ControlType.kSmartMotion,
            arbFeedforward=armFFVoltage,
            arbFFUnits=rev.SparkMaxPIDController.ArbFFUnits.kVoltage,
        )

    def _setPosition(self, position: float):
        self.lastPosition = position

    def getPosition(self) -> float:
        return self.motorEncoder.getPosition() + Arm.k_position_offset

    def setSpeed(self, speed: float):
        self.motor.set(speed)
        self._setPosition(self.motorEncoder.getPosition())

    def setInitialPosition(self):
        self._setPosition(Arm.k_position_initial)

    def setUpPosition(self):
        self._setPosition(Arm.k_position_up)

    def setDownPosition(self):
        self._setPosition(Arm.k_position_down)