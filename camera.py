import robotpy_apriltag
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

import constants
import utils.PhotonUtils as PhotonUtils


class AprilTagCamera:
    def __init__(self):
        self.camera = PhotonCamera(constants.Cameras.april_tag_camera_name)

        aprilTagField = robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2024Crescendo
        )

        self.poseEstimator = PhotonPoseEstimator(
            aprilTagField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            constants.Cameras.robot_to_april_tag_cam,
        )

    def getEstimatedPose(self, referencePose: Pose2d, robotSpeeds: ChassisSpeeds):
        self.poseEstimator.referencePose = referencePose

        if not self.camera.isConnected():
            return (None, None)

        velocityThreshold = 4
        if robotSpeeds.vx > velocityThreshold or robotSpeeds.vy > velocityThreshold:
            return (None, None)

        cameraResult = self.camera.getLatestResult()

        # check if the pose is good enough
        # the criteria could be based on the robots velocity, distance to nearest target
        if not cameraResult.hasTargets():
            return (None, None)

        latencyThreshold = 100
        if cameraResult.getLatencyMillis() > latencyThreshold:
            return (None, None)

        nearestTarget = PhotonUtils.getNearestTarget(cameraResult.getTargets())

        areaThreshold = 0.1
        if nearestTarget.getArea() < areaThreshold:
            return (None, None)

        return (self.poseEstimator.update(cameraResult), cameraResult.getTimestamp())


class NoteTrackerCamera:
    def __init__(self):
        self.camera = PhotonCamera(constants.Cameras.note_tracker_camera_name)

    def enableTracking(self):
        self.camera.setDriverMode(False)

    def disableTracking(self):
        self.camera.setDriverMode(True)

    def getNotePosition(self):
        if not self.camera.isConnected():
            return (None, None)

        res = self.camera.getLatestResult()

        if res.hasTargets():
            target = PhotonUtils.getNearestTarget(res.getTargets())

            return (target.getYaw(), target.getArea())

        return (None, None)
