#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declare the mediapipe pose detector to be used
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    # model_complexity=1
)

mp_drawing = mp.solutions.drawing_utils

def classify_gesture(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

    if pinky_tip.y < wrist.y and all(pinky_tip.y < lm.y for lm in [index_finger_tip, thumb_tip]):
        return "MOVE_RIGHT"
    elif thumb_tip.y < wrist.y and all(thumb_tip.y < lm.y for lm in [index_finger_tip, pinky_tip]):
        return "MOVE_LEFT"
    elif all(lm.y < wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):
        return "MOVE_FORWARD"
    elif all(lm.y > wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):
        return "STOP"
    else:
        return "UNKNOWN"
    
# Control message publisher
ackermann_command_publisher = None

#Operator image processing
def image_callback(msg):
    # cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Processing the image with MediaPipe

    image = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
    
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    gesture =  "UNKNOWN"
    # TODO Recognise the gesture by means of some classification from the landmarks.
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            gesture = classify_gesture(hand_landmarks)
            print("Detected gesture:", gesture)

    # TODO Draw landsmarks on the image
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display image with detected landmarks/gestures
    cv2.imshow("Hand pose Estimation", image)
    cv2.waitKey(1)

    # TODO Interpret the obtained gesture and send the ackermann control command.
    command = ackermann_msgs.msg.AckermannDrive()
    if gesture == "MOVE_FORWARD":
        command.speed = 1.0
        command.steering_angle = 0.0
    elif gesture == "MOVE_RIGHT":
        command.speed = 0.5
        command.steering_angle = 90.0
    elif gesture == "MOVE_LEFT":
        command.speed = 0.5
        command.steering_angle = -90.0
    elif gesture == "STOP":
        command.speed = 0.0
        command.steering_angle = 0.0
    else:
        command.speed = 0.0
        command.steering_angle = 0.0
    ackermann_command_publisher.publish(command)    




def main():
    global ackermann_command_publisher


    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
