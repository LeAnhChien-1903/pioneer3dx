# ROS libraries
import rclpy
from geometry_msgs.msg import PointStamped,  Point32
from sensor_msgs.msg import LaserScan, PointCloud, Imu
from nav_msgs.msg import Odometry
# Python libraries
import math
import numpy as np
# My libraries
from pioneer3dx.broadcaster import RobotBroadcaster
class SLAMController():
    def init(self, webots_ros, properties):
        self.__robot = webots_ros.robot
        self.__timeStep = 16
        rclpy.init(args=None)
        self.__node = rclpy.create_node('pioneer3dx_slam_controller')
        # Robot constants
        self.wheel_base = 0.33 # m
        self.wheel_radius = 0.0975
        self.max_speed = 12.3
        self.deltaT = 0.016
        # Variables
        self.__pose = [0.0, 0.0, 0.0] # Pose of robot x, y, theta 
        self.__prevLeftPosition = 0.0 # position of left wheel (rad)
        self.__prevRightPosition = 0.0 # position of right wheel (rad)
        self.__leftMotor = self.__robot.getDevice('left wheel')
        self.__rightMotor = self.__robot.getDevice('right wheel')
        self.__lidar = self.__robot.getDevice('lidar')
        self.__gps = self.__robot.getDevice('gps')
        self.__imu = self.__robot.getDevice('imu')
        self.__gpsData = PointStamped() # GPS data of robot
        self.__imuData = Imu()
        self.__laserScan = LaserScan() 
        self.__pointCloud = PointCloud()
        self.__odom = Odometry() 
        self.__leftVelocity = 1
        self.__rightVelocity = 1
        self.__broadcast = RobotBroadcaster(self.__node)
        # Initialize sensors and motors
        self.motorInitialization()
        self.sensorsInitialization()
        # Create a node to subscribe and publish data
        self.__gpsPub = self.__node.create_publisher(PointStamped, '/pioneer3dx/sensor/gps',10)
        self.__imuPub = self.__node.create_publisher(Imu, '/pioneer3dx/sensor/imu', 10)
        self.__laserScanPub = self.__node.create_publisher(LaserScan, '/pioneer3dx/sensor/laser_scan', 10)
        self.__pointCloudPub = self.__node.create_publisher(PointCloud, '/pioneer3dx/sensor/lidar_point_cloud', 10)  
        self.__odomPub = self.__node.create_publisher(Odometry, '/pioneer3dx/robot/odometry', 10)
    def motorInitialization(self):
        #  Motor initialization
        self.__leftMotor.setPosition(float('inf'))
        self.__rightMotor.setPosition(float('inf'))
        self.__leftMotor.setVelocity(0.0)
        self.__rightMotor.setVelocity(0.0)
        # Position sensor initialization
        self.__left_position_sensor = self.__robot.getDevice('left wheel sensor')
        self.__right_position_sensor = self.__robot.getDevice('right wheel sensor')
        self.__left_position_sensor.enable(self.__timeStep)
        self.__right_position_sensor.enable(self.__timeStep)
    def sensorsInitialization(self):
        # Lidar initialization
        self.__lidar.enable(self.__timeStep)
        self.__lidar. enablePointCloud()
        # GPS initialization
        self.__gps.enable(self.__timeStep)
        # IMU initialization
        self.__imu.enable(self.__timeStep)   
    def getGPSData(self):
        gpsData = self.__gps.getValues()
        self.__gpsData.header.stamp = self.__node.get_clock().now().to_msg()
        self.__gpsData.header.frame_id = 'base_link'
        self.__gpsData.point.x = gpsData[0]
        self.__gpsData.point.y = gpsData[1]
        self.__gpsData.point.z = 0.0
        self.__gpsPub.publish(self.__gpsData)
    def getImuData(self):
        # imuDataRPY = self.__imu.getRollPitchYaw()
        # imuDataQuaternion = self.quaternionFromRPY(imuDataRPY[0], imuDataRPY[1], imuDataRPY[2])
        imuDataQuaternion = self.__imu.getQuaternion()
        self.__imuData.header.stamp = self.__node.get_clock().now().to_msg()
        self.__imuData.header.frame_id = 'base_link'
        self.__imuData.orientation.x = imuDataQuaternion[0]
        self.__imuData.orientation.y = imuDataQuaternion[1]
        self.__imuData.orientation.z = imuDataQuaternion[2]
        self.__imuData.orientation.w = imuDataQuaternion[3]
        self.__imuPub.publish(self.__imuData)
    def getLidarLaserScan(self):
        self.__laserScan.header.stamp = self.__node.get_clock().now().to_msg()
        self.__laserScan.header.frame_id = 'lidar'
        self.__laserScan.angle_min = 2.0943949222564697
        self.__laserScan.angle_max = -2.0943949222564697
        self.__laserScan.angle_increment = -0.006289474666118622
        self.__laserScan.time_increment = 0.0
        self.__laserScan.scan_time = 0.0
        self.__laserScan.range_min = 0.05999999865889549
        self.__laserScan.range_max = 4.0960001945495605
        self.__laserScan.ranges = self.__lidar.getRangeImage()
        self.__laserScanPub.publish(self.__laserScan)
    def getLidarPointCloud(self):
        pointCloud = self.__lidar.getPointCloud()
        self.__pointCloud.header.stamp = self.__node.get_clock().now().to_msg()
        self.__pointCloud.header.frame_id = 'lidar'
        for i in range(len(pointCloud)):
            point = Point32()
            point.x = pointCloud[i].x
            point.y = pointCloud[i].y
            point.z = pointCloud[i].z
            self.__pointCloud.points.append(point)
        self.__pointCloudPub.publish(self.__pointCloud)
    def odometry(self):
        currLeftPosition = self.__left_position_sensor.getValue()
        currRightPosition = self.__right_position_sensor.getValue()
        # Compute velocity of left and right motor (m/s)
        left_wheel_est_vel = (currLeftPosition - self.__prevLeftPosition) * self.wheel_radius / self.deltaT
        right_wheel_est_vel = (currRightPosition - self.__prevRightPosition) * self.wheel_radius / self.deltaT
        # Compute linear and angular velocity of robot (m/s and rad/s)
        linear_vel = (right_wheel_est_vel + left_wheel_est_vel) / 2
        angular_vel = (right_wheel_est_vel - left_wheel_est_vel)/ self.wheel_base
        
        # Compute pose of robot (x y theta) - (m, m, m)
        self.__pose[2] += angular_vel*self.deltaT # compute theta
        vx = linear_vel * math.cos(self.__pose[2]) 
        vy = linear_vel * math.sin(self.__pose[2])
        self.__pose[0] += vx*self.deltaT
        self.__pose[1] += vy*self.deltaT
        # Publish the odometry
        self.__odom.header.stamp = self.__node.get_clock().now().to_msg()
        self.__odom.header.frame_id = "world"
        self.__odom.child_frame_id = "base_footprint"
        # Set pose
        quaternion = self.quaternion_from_euler(0.0, 0.0, self.__pose[2])
        self.__odom.pose.pose.position.x = self.__pose[0]
        self.__odom.pose.pose.position.y = self.__pose[1]
        self.__odom.pose.pose.position.z = 0.0
        self.__odom.pose.pose.orientation.x = quaternion[0]
        self.__odom.pose.pose.orientation.y = quaternion[1]
        self.__odom.pose.pose.orientation.z = quaternion[2]
        self.__odom.pose.pose.orientation.w = quaternion[3]
        # Set velocity
        self.__odom.twist.twist.linear.x =  linear_vel
        self.__odom.twist.twist.linear.y = 0.0
        self.__odom.twist.twist.linear.z = 0.0
        self.__odom.twist.twist.angular.x = 0.0
        self.__odom.twist.twist.angular.y = 0.0
        self.__odom.twist.twist.angular.z = angular_vel
        # Publish the odometry
        self.__odomPub.publish(self.__odom)
        # Broadcast the odometry
        self.__broadcast.broadcast(self.__odom)
        self.__prevLeftPosition = currLeftPosition
        self.__prevRightPosition = currRightPosition
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__leftMotor.setVelocity(self.__leftVelocity)
        self.__rightMotor.setVelocity(self.__rightVelocity)
        self.getImuData()
        self.getLidarLaserScan()
        self.odometry()
        
    @staticmethod
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=None)
    pioneer3dx = SLAMController(args=args)
    rclpy.spin(pioneer3dx)
    pioneer3dx.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main() 