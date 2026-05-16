#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class GazeboOdomTfPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_odom_tf_publisher')

        self.odom_topic = '/model/jetbot/odometry'

        self.parent_frame = 'jetbot/odom'
        self.child_frame = 'jetbot/chassis'

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('Gazebo odometry TF publisher started.')
        self.get_logger().info(f'Subscribing: {self.odom_topic}')
        self.get_logger().info(f'Publishing TF: {self.parent_frame} -> {self.child_frame}')

    def odom_callback(self, msg):
        tf_msg = TransformStamped()

        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z

        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)

    node = GazeboOdomTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()