import constants


def sgn(x: float) -> float:
    """return the sign of x"""
    return -1 if x < 0 else 1


def dz(x: float, dz=constants.Pilots.controller_deadzone):
    return x if abs(x) > dz else 0


def clamp(min: float, value: float, max: float) -> float:
    if value < min:
        return min
    if value > max:
        return max
    return value


def rotate_90_degrees_ccw(theta):
    return normalize_angle(theta - 90)


def rotate_180_degrees(theta):
    return normalize_angle(theta + 180)


def normalize_angle(theta):
    # Ensure the angle is within the range -180 to 180
    while theta <= -180:
        theta += 360
    while theta > 180:
        theta -= 360
    return theta


def calcAxisSpeedWithCurvatureAndDeadzone(
    x: float,
    c=constants.Pilots.rotationCurvature,
    b=constants.Pilots.rotationDeadzone,
    dz=constants.Pilots.controller_deadzone,
):
    """
    Calculate the speed of the axis with curvature and deadzone

    Desmos graph: https://www.desmos.com/calculator/mdgjguyiob

    :param x: the value of the axis
    :param c: the curvature of the axis
    :param b: the deadzone of the axis
    :param dz: the deadzone of the controller
    :return: output speed
    """
    if abs(x) < dz:
        return 0.0
    sign = sgn(x)
    return abs(x**c) * sign * (1 - b) + b * sign
