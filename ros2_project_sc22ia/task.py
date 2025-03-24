# Project

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        # Initialise any flags that signal a colour has been detected (default to false)
        self.red_found = False
        self.green_found = False
        self.blue_found = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.move_forward_message = 'move forward'
        self.stop_message = '000'
        
        self.moveBackwardsFlag = False
        self.moveForwardsFlag = False
        self.stopMovingFlag = False

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def callback(self, data):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError:
            print("Exception occured when converting image.")

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Set the upper and lower bounds to identify red, green and blue
        hsv_red_lower_1 = np.array([0, 100, 100])
        hsv_red_upper_1 = np.array([0 + self.sensitivity, 255, 255])
        
        hsv_red_lower_2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper_2 = np.array([180, 255, 255])

        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Filter out everything but particular colours using the cv2.inRange() method
        red_mask_1 = cv2.inRange(Hsv_image, hsv_red_lower_1, hsv_red_upper_1)
        red_mask_2 = cv2.inRange(Hsv_image, hsv_red_lower_2, hsv_red_upper_2)
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        green_mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)

        blue_mask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rg_mask, blue_mask)

        # TODO: Detect red and green as well

        # Find the contours that appear within the red mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, _ = cv2.findContours(red_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop over the contours
        if len(contours)>0:
            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)

            # Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 1: #<What do you think is a suitable area?>
                # Alter the value of the flag
                self.red_found = True
                print("Flag (red found):", self.red_found, cv2.contourArea(c))

        # Find the contours that appear within the green mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop over the contours
        if len(contours)>0:
            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)

            # Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 1: #<What do you think is a suitable area?>
                # Alter the value of the flag
                self.green_found = True
                print("Flag (green found):", self.green_found, cv2.contourArea(c))

        # Find the contours that appear within the blue mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop over the contours
        if len(contours)>0:
            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)

            # Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 1: #<What do you think is a suitable area?>
                # Alter the value of the flag
                self.blue_found = True
                print("Flag (blue found):", self.blue_found, cv2.contourArea(c))

            # Check if a flag has been set = colour object detected - follow the colour object
            if self.blue_found == True:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100000:
                    # Close to object, need to stop
                    # Set a flag to tell the robot to stop when in the main loop
                    self.stopMovingFlag = True
                    print("Time to stop")
                    
                elif cv2.contourArea(c) < 1:
                    # Far away from object, need to move forwards
                    # Set a flag to tell the robot to move forwards when in the main loop
                    self.moveForwardsFlag = True
                    print("Time to move forwards")

        # Show the image
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)

    def walk_forward(self):
        # Make the robot move forwards
        desired_velocity = Twist()

        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s
        
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        # Make the robot move backwards
        desired_velocity = Twist()

        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        # Make the robot stop
        desired_velocity = Twist()

        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        
        self.publisher.publish(desired_velocity)

    # Add functions to go to specific point on map
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.

        ## Access the current pose
        #current_pose = feedback_msg.feedback.current_pose
        #position = current_pose.pose.position
        #orientation = current_pose.pose.orientation

        ## Access other feedback fields
        #navigation_time = feedback_msg.feedback.navigation_time
        #distance_remaining = feedback_msg.feedback.distance_remaining

        ## Print or process the feedback data
        #self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        #self.get_logger().info(f'Distance Remaining: {distance_remaining}')

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args=None):
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=args)
    robot = Robot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Publish moves
            # Check if a blue box has been detected
            if robot.blue_found == True:
                # Check if the robot should stop
                if robot.stopMovingFlag == True:
                    print("Going to stop")
                    robot.stop()
                # Check if the robot should move forward
                elif robot.moveForwardsFlag == True:
                    print("Going to walk forward")
                    robot.walk_forward()
            else:
            
                # TODO: Add other movement

                robot.walk_forward()
                robot.send_goal(-8.67, -5.88, 0.00247)
                #rclpy.spin(robot)

    except ROSInterruptException:
        pass

    # Destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
