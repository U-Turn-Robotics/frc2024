#!/usr/bin/env python3

import sys
import time

import ntcore

args = sys.argv[1:]

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("SmartDashboard")
    xSub = table.getDoubleTopic("x").subscribe(0)
    y = 0.0
    ySub = table.getDoubleTopic("y").publish()
    inst.startClient4("example client")
    if "local" in args:
        inst.setServer("localhost")
    else:
        inst.setServerTeam(9623)
    inst.startDSClient()  # recommended if running on DS computer; this gets the robot IP from the DS

    while True:
        time.sleep(1)

        x = xSub.get()
        ySub.set(y)
        y += 1
        print(f"X: {x} Y: {y}")

# import mediapipe as mp

# mp_drawing = mp.solutions.drawing_utils
# mp_holistic = mp.solutions.holistic

# if __name__ == "__main__":
#     inst = ntcore.NetworkTableInstance.getDefault()
#     table = inst.getTable("SmartDashboard")
#     xSub = table.getDoubleTopic("x").subscribe(0)
#     y = 0.0
#     ySub = table.getDoubleTopic("y").publish()
#     inst.startClient4("example client")
#     if "local" in args:
#         inst.setServer("localhost")
#     else:
#         inst.setServerTeam(9623)
#     inst.startDSClient()  # recommended if running on DS computer; this gets the robot IP from the DS

#     cap = cv2.VideoCapture(0)
#     with mp_holistic.Holistic(
#         min_detection_confidence=0.5, min_tracking_confidence=0.5
#     ) as holistic:
#         while cap.isOpened():
#             ret, frame = cap.read()
#             if not ret:
#                 break

#             # Convert the BGR image to RGB.
#             image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # To improve performance, optionally mark the image as not writeable to pass by reference.
#             image.flags.writeable = False

#             # Process the image.
#             results = holistic.process(image)

#             # Draw landmarks on the image.
#             image.flags.writeable = True
#             image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#             mp_drawing.draw_landmarks(
#                 image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS
#             )
#             mp_drawing.draw_landmarks(
#                 image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS
#             )
#             mp_drawing.draw_landmarks(
#                 image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS
#             )
#             mp_drawing.draw_landmarks(
#                 image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS
#             )

#             # Get the coordinates of the elbow.
#             if results.pose_landmarks:
#                 left_elbow = results.pose_landmarks.landmark[
#                     mp_holistic.PoseLandmark.LEFT_ELBOW
#                 ]
#                 right_elbow = results.pose_landmarks.landmark[
#                     mp_holistic.PoseLandmark.RIGHT_ELBOW
#                 ]

#                 # Calculate the angle of the elbow.
#                 left_elbow_angle = calculate_angle(
#                     left_elbow,
#                     results.pose_landmarks.landmark[
#                         mp_holistic.PoseLandmark.LEFT_WRIST
#                     ],
#                     results.pose_landmarks.landmark[
#                         mp_holistic.PoseLandmark.LEFT_SHOULDER
#                     ],
#                 )
#                 right_elbow_angle = calculate_angle(
#                     right_elbow,
#                     results.pose_landmarks.landmark[
#                         mp_holistic.PoseLandmark.RIGHT_WRIST
#                     ],
#                     results.pose_landmarks.landmark[
#                         mp_holistic.PoseLandmark.RIGHT_SHOULDER
#                     ],
#                 )

#                 # Update the network tables with the elbow angles.
#                 table.putNumber("left_elbow_angle", left_elbow_angle)
#                 table.putNumber("right_elbow_angle", right_elbow_angle)

#             # Display the image.
#             cv2.imshow("MediaPipe Holistic", image)

#             if cv2.waitKey(10) & 0xFF == ord("q"):
#                 break

#     cap.release()
#     cv2.destroyAllWindows()
