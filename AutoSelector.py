import contextlib
import json
import os
import typing

import wpilib as wp
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from wpimath.geometry import Pose2d


class AutoSelector:
    def __init__(self):
        self.autoSelector = wp.SendableChooser()
        autos = AutoSelector._findAutos()

        # TODO make a better way to find the default auto
        # by getting the pose of the robot from the camera as it's disabled, use the auto with the closest starting pose
        (autoName, (_, defaultAuto)) = self.getClosestAuto(autos, Pose2d(0, 0, 0))
        self.autoSelector.setDefaultOption(autoName, defaultAuto)

        for autoName, auto in autos.items():
            if auto is defaultAuto:
                continue
            self.autoSelector.addOption(autoName, auto)

        wp.SmartDashboard.putData("Auto Selector", self.autoSelector)

        self.lastAutoSelected = None

    @staticmethod
    def _findAutos() -> typing.Dict[str, typing.Tuple[Pose2d, PathPlannerAuto]]:
        autosPath = os.path.join(wp.getDeployDirectory(), "pathplanner", "autos")

        autos = {}
        for file in os.listdir(autosPath):
            if file.endswith(".auto"):
                with open(os.path.join(autosPath, file)) as f:
                    auto_json = json.loads(f.read())
                    starting_pose = None
                    with contextlib.suppress(KeyError):
                        starting_pose = AutoBuilder.getStartingPoseFromJson(
                            auto_json["startingPose"]
                        )
                    autos[file.split(".auto")[0]] = (
                        starting_pose,
                        AutoBuilder.getAutoCommandFromJson(auto_json),
                    )

        return autos

    def getClosestAuto(
        self,
        autos: typing.Dict[str, typing.Tuple[Pose2d, PathPlannerAuto]],
        pose: Pose2d,
    ) -> typing.Tuple[str, typing.Tuple[Pose2d | None, PathPlannerAuto]]:
        closestAuto = None
        closestDistance = float("inf")
        for autoName, auto in autos.items():
            autoPose = auto[0]
            if autoPose is None:
                continue
            distance = pose.translation().distance(autoPose.translation())
            if distance < closestDistance:
                closestDistance = distance
                closestAuto = (autoName, auto)
        return closestAuto

    def getSelectedAuto(self) -> typing.Tuple[Pose2d | None, PathPlannerAuto]:
        return self.autoSelector.getSelected()
