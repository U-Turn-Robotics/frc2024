from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
import robotpy_apriltag
from wpimath.geometry import Translation3d, Transform3d, Rotation3d, Pose2d
from wpimath.kinematics import ChassisSpeeds


class AprilTagCamera:
    def __init__(self) -> None:
        self.camera = PhotonCamera("photonvision")

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
            return None

        cameraResult = self.camera.getLatestResult()

        # check if the pose is good enough
        # the criteria could be based on the robots velocity, distance to nearest target
        if not cameraResult.hasTargets():
            return None
        nearestTarget = None
        for target in cameraResult.getTargets():
            if nearestTarget is None:
                nearestTarget = target
                continue
            if target.getArea() > nearestTarget.getArea():
                nearestTarget = target

        areaThreshold = 0.1
        if nearestTarget.getArea() < areaThreshold:
            return None

        return self.poseEstimator.update(cameraResult)
