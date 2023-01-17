import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class RobotBroadcaster():
    def __init__(self, node):
        self.__node = node
        self.tf_broadcaster = TransformBroadcaster(self.__node)
    def broadcast(self, odometry):
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = odometry.header.frame_id
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation.x = odometry.pose.pose.orientation.x
        transform.transform.rotation.y = odometry.pose.pose.orientation.y
        transform.transform.rotation.z = odometry.pose.pose.orientation.z
        transform.transform.rotation.w = odometry.pose.pose.orientation.w

        # self.tf_broadcaster.sendTransform(transform)
        # transform = TransformStamped()
        # transform.header.stamp = self.__node.get_clock().now().to_msg()
        # transform.header.frame_id = 'base_link'
        # transform.child_frame_id = 'lidar'
        # transform.transform.translation.x = 0.1
        # transform.transform.translation.y = 0.0
        # transform.transform.translation.z = 0.19
        # transform.transform.rotation.x = 0.0
        # transform.transform.rotation.y = 0.0
        # transform.transform.rotation.z = 0.0
        # transform.transform.rotation.w = 1.0

        # self.tf_broadcaster.sendTransform(transform)
        # # base_link and base foot_print
        # transform = TransformStamped()
        # transform.header.stamp = self.__node.get_clock().now().to_msg()
        # transform.header.frame_id = 'base_footprint'
        # transform.child_frame_id = 'base_link'
        # transform.transform.translation.x = 0.0
        # transform.transform.translation.y = 0.0
        # transform.transform.translation.z = 0.0957809
        # transform.transform.rotation.x = 0.0
        # transform.transform.rotation.y = 0.0
        # transform.transform.rotation.z = 0.0
        # transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
