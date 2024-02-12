from typing import TYPE_CHECKING

from pyfrc.physics.core import PhysicsEngine as Engine
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import AnalogGyroSim
from wpimath.geometry import Pose2d, Rotation2d

if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine(Engine):
    simGyro = AnalogGyroSim(0)

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        """Initialize the physics engine with the simulation interface"""

        self.robot = robot
        self.physics_controller = physics_controller

        assert self.physics_controller.field is not None
        self.physics_controller.field.setRobotPose(Pose2d(5, 5, Rotation2d(0)))

    def update_sim(self, now: float, tm_diff: float):
        """Called when the simulation parameters for the program need to be updated"""

        assert self.physics_controller.field is not None
        self.physics_controller.field.setRobotPose(Pose2d(5, 5, Rotation2d(0)))
        self.simGyro.setAngle(90)
