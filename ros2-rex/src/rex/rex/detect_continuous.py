#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
from time import sleep

class ContinuousTrackletsParser(Node):

    def __init__(self):
        super().__init__('continuous_tracklets_parser')
        self.get_logger().info('Initialising ContinuousTrackletsParser node...')
        
        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/color/tracklets',
            self.listener_callback,
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscription to /color/tracklets created.')
        
        self.previous_tracking_status = {}
        self.track_confirmations = {}
        
        self.tracked_positions = []
        self.confirmation_threshold = 5

    def listener_callback(self, msg):
        for detection in msg.detections:
            tracking_id = detection.tracking_id
            tracking_status = detection.tracking_status
            previous_status = self.previous_tracking_status.get(tracking_id, None)
            
            if previous_status == 2 and tracking_status == 2:
                continue
            
            self.previous_tracking_status[tracking_id] = tracking_status

            for result in detection.results:
                if result.hypothesis.class_id == '15':
                    position = result.pose.pose.position

                    if tracking_status == 1:
                        self.track_confirmations[tracking_id] = self.track_confirmations.get(tracking_id, 0) + 1
                        if self.track_confirmations[tracking_id] >= self.confirmation_threshold:
                            self.track_confirmations[tracking_id] = 0  # Reset counter after reaching threshold
                            real_world_position = self.transform_position(position.x, position.z)  # X is horizontal, Z is depth
                            self.tracked_positions.append(real_world_position)
                            self.get_logger().info(f'Confirmed position - x: {real_world_position[0]:.2f}, y: {real_world_position[1]:.2f} - Tracking({tracking_id}): Yes')

                    self.get_logger().info(f'Detection - x: {position.x:.2f}, z: {position.z:.2f} - Tracking({tracking_id}): {"Yes" if tracking_status == 1 else "No"}')

    def transform_position(self, x, z):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform('odom', 'ldlidar_base', rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            
            # Convert quaternion to rotation matrix
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
            rot_matrix = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            
            # Camera coordinates to robot coordinates
            camera_coords = np.array([z, -x, 0])  # Camera X -> -Robot Y (flipped), Camera Z -> Robot X
            robot_coords = np.dot(rot_matrix, camera_coords) + np.array([translation.x, translation.y, translation.z])
            
            return robot_coords[:2]  # Return only x and y, ignore z
        except Exception as e:
            self.get_logger().error(f'Error transforming position: {e}')
            return np.array([0.0, 0.0])

    def publish_goal(self):
        if self.tracked_positions:
            goal_position = self.tracked_positions[-1]
            goal_position[0] -= 0.3  # Set goal 30 cm (0.3 m) in front of the detected position
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'odom'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_position[0]
            goal_pose.pose.position.y = goal_position[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0  # Assuming no rotation

            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f'Goal position published - x: {goal_position[0]:.2f}, y: {goal_position[1]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    tracklets_parser = ContinuousTrackletsParser()

    while rclpy.ok():
        rclpy.spin_once(tracklets_parser, timeout_sec=1)  # Spin to keep node running and processing detections

        if tracklets_parser.tracked_positions:
            sleep(2)  # Pause for 2 seconds before sending the goal
            tracklets_parser.publish_goal()

    tracklets_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
