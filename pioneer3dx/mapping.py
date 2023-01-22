import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations  import euler_from_quaternion
import cv2
MAPS_PATH =  '/home/leanhchien/webots_ws/src/pioneer3dx/maps'
MAP_NAME = 'pioneer3dx_world'
from pioneer3dx.grid_map import *

class GMapping(Node):
    def __init__(self, map_x_lim = [0, 30], map_y_lim = [0, 30], P_prior = 0.5, P_occ = 0.9, P_free = 0.3, resolution = 0.1):
        super().__init__('mapping')
        self.map_x_lim = map_x_lim
        self.map_y_lim = map_y_lim 
        self.P_prior = P_prior # Prior occupancy probability
        self.P_occ = P_occ     # Probability that cell is occupied with total confidence
        self.P_free = P_free   # Probability that cell is free with total confidence 
        self.resolution = resolution # Grid resolution [m]
        self.gridMap = GridMap(map_x_lim, map_y_lim, resolution, P_prior) # Create grid map
        self.laserScan = LaserScan()
        self.odometry = Odometry()
        self.map = OccupancyGrid()
        self.mapPub = self.create_publisher(OccupancyGrid, '/pioneer3dx/map/occupancy', 10)
        self.create_subscription(LaserScan, "/pioneer3dx/sensor/laser_scan", self.laserScanCallback, 10)
        self.create_subscription(Odometry, '/pioneer3dx/wheel/odometry', self.odometryCallback, 10)
        self.create_timer(0.016, self.timerCallback)
        
    def timerCallback(self):
        print('Start mapping')
        distances, angles, information = self.convertLaserScan()
        x_odom, y_odom, theta_odom = self.get_odom_pose()
        distances_x, distances_y = self.lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)
        x1, y1 = self.gridMap.discretize(x_odom, y_odom)
        # for BGR image of the grid map
        X2 = []
        Y2 = []
        for (dist_x, dist_y, dist) in zip(distances_x, distances_y, distances):
            x2, y2 = self.gridMap.discretize(dist_x, dist_y)
            for (x_bres, y_bres)in bresenham(self.gridMap, x1, y1, x2, y2):
                self.gridMap.update(x = x_bres, y = y_bres, p = self.P_free)
            if dist < 4.095:
                self.gridMap.update(x = x2, y = y2, p = self.P_occ)
            X2.append(x2)
            Y2.append(y2)
        grayscale_image = self.gridMap.to_grayscale_image()
        result_image = cv2.rotate(src = grayscale_image, rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = 1 - cv2.rotate(src = grayscale_image, rotateCode = cv2.ROTATE_180)
        cv2.imwrite(img = result_image * 255, filename =  MAPS_PATH + '/' + MAP_NAME +'_grid_map.png')
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.resolution
        self.map.info.width = int((self.map_x_lim[1] - self.map_y_lim[0])/self.resolution) + 1
        self.map.info.height = int((self.map_y_lim[1] - self.map_y_lim[0])/self.resolution) + 1
        self.map.info.origin = self.odometry.pose.pose
        self.map.data = np.uint8(np.ndarray.flatten(image*100, order='F'))[::-1].tolist()
        self.mapPub.publish(self.map)    
    def laserScanCallback(self, msg):
        self.laserScan = msg
    def odometryCallback(self, msg):
        self.odometry = msg
    def convertLaserScan(self):
        """
            Convert LaserScan to array
        """
        distances = np.array([])
        information = np.array([])
        angles = np.arange(start=self.laserScan.angle_min, stop=self.laserScan.angle_max, step = self.laserScan.angle_increment)
        for i in range(len(self.laserScan.ranges)):
            # distance calculation
            if (self.laserScan.ranges[i] > self.laserScan.range_max):
                distance = self.laserScan.range_max
            elif (self.laserScan.ranges[i] < self.laserScan.range_min):
                distance = self.laserScan.range_min
            else:
                distance = self.laserScan.ranges[i]
            # smaller the distance, bigger the information (measurement is more confident)
            info = ((self.laserScan.range_max - distance) / self.laserScan.range_max)**2
        
            distances = np.append(distances, distance)
            information = np.append(information, info)

        # distances in [m], angles in [radians], information [0-1]
        return ( distances, angles, information )
    def get_odom_pose(self):
        """
            Get (x,y, theta) coordinates from Odometry msg in [m]
        """
        orientation_q = self.odometry.pose.pose.orientation
        theta = self.transform_orientation(orientation_q)
        # self.odometry.pose.pose.position.x = self.odometry.pose.pose.position.x + 0.1 * np.cos(theta)
        # self.odometry.pose.pose.position.y = self.odometry.pose.pose.position.y + 0.1 * np.sin(theta)
        x = self.odometry.pose.pose.position.x + int(self.map_x_lim[1]/2)
        y = self.odometry.pose.pose.position.y + int(self.map_y_lim[1]/2)
        return (x, y, theta)
        
    @staticmethod
    def lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom):
        """
            Lidar measurement in X-Y planes
        """
        distances_x = np.array([])
        distances_y = np.array([])
        
        for (dist, ang) in zip(distances, angles):
            distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))
            distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

        return (distances_x, distances_y)
    @staticmethod
    def transform_orientation(orientation_q):
        """
            Transform theta to [radians] from [quaternion orientation]
        """
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_ , _, yaw) = euler_from_quaternion(orientation_list)
        if yaw < 0:
            yaw = 2*np.pi + yaw
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    gmapping = GMapping()
    rclpy.spin(gmapping)
    gmapping.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
