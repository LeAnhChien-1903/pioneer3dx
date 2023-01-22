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
        self.tf_broadcaster.sendTransform(transform)
