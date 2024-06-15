#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray

class TrackletsParser(Node):

    def __init__(self):
        super().__init__('tracklets_parser')
        self.get_logger().info('Initializing TrackletsParser node...')
        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/color/tracklets',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscription to /color/tracklets created.')
        self.previous_tracking_status = {}

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

                    x = f'{position.x:.2f}'
                    y = f'{position.y:.2f}'
                    z = f'{position.z:.2f}'

                    self.get_logger().info(f'Position - x: {x}, y: {y}, z: {z} - Tracking({tracking_id}): {"Yes" if tracking_status == 1 else "No"}')

def main(args=None):
    rclpy.init(args=args)
    tracklets_parser = TrackletsParser()
    rclpy.spin(tracklets_parser)
    tracklets_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
