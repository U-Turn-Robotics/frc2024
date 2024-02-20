import ntcore
import robotpy_apriltag
from cscore import CameraServer
from wpimath.geometry import Pose2d, Transform2d

import cv2
import numpy as np


#
# This code will work both on a RoboRIO and on other platforms. The exact mechanism
# to run it differs depending on whether you’re on a RoboRIO or a coprocessor
#
# https://robotpy.readthedocs.io/en/stable/vision/code.html


def main():
    detector = robotpy_apriltag.AprilTagDetector()

    # Look for tag36h11, correct 3 error bits
    detector.addFamily("tag36h11", 3)

    # Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    # (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    poseEstConfig = robotpy_apriltag.AprilTagPoseEstimator.Config(
        0.1651,
        699.3778103158814,
        677.7161226393544,
        345.6059345433618,
        207.12741326228522,
    )
    estimator = robotpy_apriltag.AprilTagPoseEstimator(poseEstConfig)

    # Get the UsbCamera from CameraServer
    camera = CameraServer.startAutomaticCapture()

    # Set the resolution
    camera.setResolution(640, 480)

    # Get a CvSink. This will capture Mats from the camera
    cvSink = CameraServer.getVideo()

    # Set up a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("Detected", 640, 480)

    # Mats are very memory expensive. Let's reuse these.
    mat = np.zeros((480, 640, 3), dtype=np.uint8)
    grayMat = np.zeros(shape=(480, 640), dtype=np.uint8)

    # Instantiate once
    tags = []  # The list where the tags will be stored
    outlineColor = (0, 255, 0)  # Color of Tag Outline
    crossColor = (0, 0, 25)  # Color of Cross

    # Output the list to Network Tables
    tagsTable = ntcore.NetworkTableInstance.getDefault().getTable("apriltags")
    pubTags = tagsTable.getIntegerArrayTopic("tags").publish()

    aprilTagField = robotpy_apriltag.loadAprilTagLayoutField(
        robotpy_apriltag.AprilTagField.k2024Crescendo
    )

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source mat.  If there is an error notify the output.
        if cvSink.grabFrame(mat) == 0:
            # Send the output frame the error
            outputStream.notifyError(cvSink.getError())

            # Skip the rest of the current iteration
            continue

        cv2.cvtColor(mat, cv2.COLOR_RGB2GRAY, dst=grayMat)

        detections = detector.detect(grayMat)

        tags.clear()

        bestDetection, bestDetectionMargin = (
            (detections[0], detections[0].getDecisionMargin())
            if len(detections) > 0
            else (None, float("-inf"))
        )
        for detection in detections[1:]:
            margin = detection.getDecisionMargin()
            if margin > bestDetectionMargin:
                bestDetection = detection
                bestDetectionMargin = margin

        if bestDetection:

            # Remember the tag we saw
            tags.append(bestDetection.getId())

            # Draw lines around the tag
            for i in range(4):
                j = (i + 1) % 4
                point1 = (
                    int(bestDetection.getCorner(i).x),
                    int(bestDetection.getCorner(i).y),
                )
                point2 = (
                    int(bestDetection.getCorner(j).x),
                    int(bestDetection.getCorner(j).y),
                )
                mat = cv2.line(mat, point1, point2, outlineColor, 2)

            # Mark the center of the tag
            cx = int(bestDetection.getCenter().x)
            cy = int(bestDetection.getCenter().y)
            ll = 10
            mat = cv2.line(
                mat,
                (cx - ll, cy),
                (cx + ll, cy),
                crossColor,
                2,
            )
            mat = cv2.line(
                mat,
                (cx, cy - ll),
                (cx, cy + ll),
                crossColor,
                2,
            )

            # Identify the tag
            mat = cv2.putText(
                mat,
                str(bestDetection.getId()),
                (cx + ll, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3,
            )

            # Determine Tag Pose, relative to robot
            tagTransform = estimator.estimate(bestDetection)
            tagRelativePose = Pose2d(tagTransform, tagTransform.rotation())
            tagRelativePose = tagRelativePose * -1

            tagFieldPose = aprilTagField.getTagPose(bestDetection.getId()).toPose2d()
            tagFieldTransformation = Transform2d(tagFieldPose.x, tagFieldPose.y, 0)
            robotPose = tagRelativePose.transformBy(tagFieldTransformation)

            tagsTable.getEntry("pose").setDoubleArray(
                [robotPose.x, robotPose.y, robotPose.rotation().radians()]
            )
            # put pose into dashboard
            # rot = pose.rotation()
            # tagsTable.getEntry(f"pose_{detection.getId()}").setDoubleArray(
            #     [pose.X(), pose.Y(), pose.Z(), rot.X(), rot.Y(), rot.Z()]
            # )

        # Put List of Tags onto Dashboard
        pubTags.set(tags)

        # Give output stream a new image to display
        outputStream.putFrame(mat)

    # The camera code will be killed when the robot.py program exits. If you wish to perform cleanup,
    # you should register an atexit handler. The child process will NOT be launched when running the robot code in
    # simulation or unit testing mode
