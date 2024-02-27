import math

from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.units import inchesToMeters


class Pilots:
    k_driver_controller_port = 0
    k_operator_controller_port = 1

    rotationScale = 0.5

    rotationCurvature = 2.0
    rotationDeadzone = 0.075

    controller_deadzone = 0.07


class NeoConstants:
    k_units_per_revolution = 42


class Drivetrain:
    speed_scale = 1

    k_left_motor1_port = 2
    k_left_motor2_port = 3
    k_right_motor1_port = 4
    k_right_motor2_port = 5

    k_motor_reduction = 8.45
    k_wheel_diameter_meters = inchesToMeters(6)  # 0.1524
    kWheelRadiusMeters = k_wheel_diameter_meters / 2

    k_wheel_circumference_meters = k_wheel_diameter_meters * math.pi
    k_encoder_pulses_per_revolution = (
        k_motor_reduction * NeoConstants.k_units_per_revolution
    )
    k_encoder_position_per_meter = (
        k_encoder_pulses_per_revolution / k_wheel_circumference_meters
    )

    k_position_conversion_factor = (
        NeoConstants.k_units_per_revolution / k_encoder_position_per_meter
    )  # manually calculated as 0.056148

    k_dt_current_limit = 60

    # TODO: Tune
    k_forward_p = 0.012
    k_forward_i = 0.0
    k_forward_d = 0.0005

    k_turn_p = 0.012
    k_turn_i = 0.0
    k_turn_d = 0.0005
    k_turn_max_speed = 0.5

    # TODO: Tune
    k_max_velocity = 3.0

    differential_drive_kinematics = DifferentialDriveKinematics(inchesToMeters(23.5))


class Robot:
    period = 0.02


class Arm:
    k_motor_id = 9

    # TODO: Tune all
    k_p = 0.1
    k_i = 0
    k_d = 0

    k_max_velocity = 1000
    k_max_acceleration = 500

    k_s = 1.0
    k_g = 9.65
    k_v = 0.94
    k_a = 0.7

    k_position_initial = math.radians(45)
    k_position_up = math.radians(80)
    k_position_down = math.radians(0)
    k_position_offset = k_position_initial
    k_preset_positions = [k_position_initial, k_position_up, k_position_down]

    k_gear_reduction = 48
    k_encoder_pulses_per_revolution = (
        NeoConstants.k_units_per_revolution * k_gear_reduction
    )
    k_encoder_position_per_degree = k_encoder_pulses_per_revolution / 360
    k_encoder_position_per_radian = math.radians(k_encoder_position_per_degree)


class Shooter:
    k_motor_1_id = 6
    k_motor_2_id = 7

    k_shoot_speed = 1

    # TODO tune
    k_shooter_ready_velocity = 4


class Pickup:
    k_motor_id = 8
    k_pickup_speed = 1
