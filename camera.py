import math

import robotpy_apriltag
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Pose2d, Rotation3d, Transform3d, Translation3d
from wpimath.kinematics import ChassisSpeeds

import PhotonUtils


class AprilTagCamera:
    def __init__(self):
        self.camera = PhotonCamera("picam")

        aprilTagField = robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2024Crescendo
        )

        # Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        robotToCam = Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0, 0, 0))

        self.poseEstimator = PhotonPoseEstimator(
            aprilTagField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            robotToCam,
        )

    def getEstimatedPose(self, referencePose: Pose2d, robotSpeeds: ChassisSpeeds):
        self.poseEstimator.referencePose = referencePose

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
        self.camera = PhotonCamera("usb_cam")

    def enableTracking(self):
        self.camera.setDriverMode(False)

    def disableTracking(self):
        self.camera.setDriverMode(True)

    def getNotePosition(self):
        res = self.camera.getLatestResult()

        if res.hasTargets():
            target = PhotonUtils.getNearestTarget(res.getTargets())

            return (target.getYaw(), target.getArea())

        return (None, None)
