import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FindWall
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math

class WallFinderNode(Node):
    def __init__(self):
        super().__init__('wall_finder_node')
        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.srv = self.create_service(FindWall,
             '/find_wall', 
             self.find_wall_callback,
             callback_group=self.reentrant_group_1)
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel',10)
        self.forward_speed = 0.1
        self.turning_speed = 0.0
        self.intial_orientation = True
        self.start_simulation = True
        self.closest_sector = None
        self.lowest_distance = None
        
        

    def laser_callback(self, msg):
        total_beams = len(msg.ranges)
        if total_beams <= 1:
            return

        angle_span_rad = msg.angle_max - msg.angle_min

        angle_span_deg = math.degrees(angle_span_rad)
        if angle_span_deg <= 0:
            return
        idx_per_deg = total_beams / angle_span_deg

        start_angle_deg = math.degrees(msg.angle_min)

        def deg_to_idx(target_deg):
            raw_diff = target_deg - start_angle_deg
            relative_deg = raw_diff % 360.0
            idx = int(relative_deg * idx_per_deg)
            return max(0, min(idx, total_beams - 1))

        sectors = {
            'front':            list(range(deg_to_idx(0), deg_to_idx(31))) + list(range(deg_to_idx(330), deg_to_idx(360))),
            'side_front_left':  list(range(deg_to_idx(30), deg_to_idx(90))), 
            'side_rear_left':   list(range(deg_to_idx(90), deg_to_idx(150))), 
            'rear':             list(range(deg_to_idx(150), deg_to_idx(210))), 
            'side_rear_right':  list(range(deg_to_idx(210), deg_to_idx(270))), 
            'side_front_right': list(range(deg_to_idx(270), deg_to_idx(330)))
        }
        
        #self.get_logger().info(f"LiDAR array length: {len(msg.ranges)}")

        min_distances = {key: float('inf') for key in sectors}
        for sector, ranges in sectors.items():
            #sector_ranges = [msg.ranges[i] for i in ranges if 0.0 < msg.ranges[i] < float('inf')]
            sector_ranges = [
                    msg.ranges[i] for i in ranges 
                    if not math.isnan(msg.ranges[i]) and 0.18 < msg.ranges[i] < msg.range_max
                    ]
            if sector_ranges:
                min_distances[sector] = min(sector_ranges)
        min_distances_around = min_distances

        right_indices = list(range(deg_to_idx(265), deg_to_idx(275)))
        valid_right = [msg.ranges[i] for i in right_indices if msg.range_min < msg.ranges[i] < msg.range_max]
        self.distance_to_right = sum(valid_right) / len(valid_right) if valid_right else float('inf')

        self.closest_sector = min(min_distances_around, key=min_distances_around.get)
        self.lowest_distance = min_distances_around[self.closest_sector]
        #self.get_logger().info(f'Closest sector is {self.closest_sector} with distance = {self.lowest_distance}')
        

    def publish_velocity(self, linear_velocity, angular_velocity):
        msg_cmd = Twist()
        msg_cmd.linear.x = linear_velocity
        msg_cmd.angular.z = angular_velocity
        self.publisher_cmd.publish(msg_cmd)

    def find_wall_callback(self, request, response):

        wall_found = False
        while not wall_found:
            if self.closest_sector == 'front':
                self.get_logger().info(f'inside closest sector = front with {self.lowest_distance}')
                if self.lowest_distance > 0.2:
                    self.publish_velocity(0.05,0.0)
                else:
                    self.get_logger().info(f'!!!!! {wall_found}')
                    wall_found = True
            else:
                self.publish_velocity(0.0,0.5)

            time.sleep(0.05)

        while wall_found and self.distance_to_right > 0.3:
            self.publish_velocity(0.0,0.5)
            self.get_logger().info(f'>>distance_to_right: {self.distance_to_right}')

            time.sleep(0.05)

        self.publish_velocity(0.0,0.0)
        self.get_logger().info(f'Wall found is {wall_found}')
        response.wall_found = wall_found
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WallFinderNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()