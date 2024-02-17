class Pilots:
    k_driver_controller_port = 0
    k_operator_controller_port = 1

    rotationScale = 0.5

    rotationCurvature = 2.0
    rotationDeadzone = 0.075

    controller_deadzone = 0.07


import math

from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.units import inchesToMeters


class NeoConstants:
    k_units_per_revolution = 42


class Drivetrain:
    speed_scale = 0.2

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
    k_turn_p = 0.012
    k_turn_i = 0.0
    k_turn_d = 0.0005
    k_turn_max_speed = 0.5

    # TODO: Tune
    k_max_velocity = 3.0

    differential_drive_kinematics = DifferentialDriveKinematics(inchesToMeters(23.5))


class Robot:
    period = 0.02


# # Launcher
# k_feeder_motor = 5
# k_launcher_motor = 6
# k_launcher_current_limit = 80
# k_feed_current_limit = 80

# k_launcher_speed = 1
# k_launch_feeder_speed = 1
# k_intake_launcher_speed = -1
# k_intake_feeder_speed = -0.2
# k_launcher_delay = 1
