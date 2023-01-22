from tf_transformations  import euler_from_quaternion
import cv2
MAPS_PATH =  '/home/leanhchien/webots_ws/src/pioneer3dx/maps'
MAP_NAME = 'pioneer3dx_world'
from pioneer3dx.grid_map import *
from pioneer3dx.utlis import *


map_x_lim = [0, 30]
map_y_lim = [0, 30]
P_prior = 0.5	# Prior occupancy probability
P_occ = 0.9	# Probability that cell is occupied with total confidence
P_free = 0.3	# Probability that cell is free with total confidence 


x_odom = 5
y_odom = 13
theta_odom = np.pi/3
distances = np.ones(667)*2
angles = np.arange(start=2*np.pi/3, stop = -2*np.pi/3, step = -4*np.pi/(3*667))
information= ((4.095 - distances) / 4.095)**2
distances_x, distances_y = lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)

gridMap = GridMap(X_lim = map_x_lim, 
				  Y_lim = map_y_lim, 
				  resolution = 0.1, 
				  p = P_prior)
x1, y1 = gridMap.discretize(x_odom, y_odom)
# for BGR image of the grid map
X2 = []
Y2 = []
for (dist_x, dist_y, dist) in zip(distances_x, distances_y, distances):
    x2, y2 = gridMap.discretize(dist_x, dist_y)
    for (x_bres, y_bres) in bresenham(gridMap, x1, y1, x2, y2):
        gridMap.update(x = x_bres, y = y_bres, p = P_free)
    if dist < 4.095:
        gridMap.update(x = x2, y = y2, p = P_occ)
        # for BGR image of the grid map
    X2.append(x2)
    Y2.append(y2)

grayscale_image = gridMap.to_grayscale_image()
rotated_image = cv2.rotate(src = grayscale_image, rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)
cv2.imshow("Grid map", rotated_image)

cv2.waitKey()