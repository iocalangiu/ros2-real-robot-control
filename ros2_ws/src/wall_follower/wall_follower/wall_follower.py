#!/usr/bin/env/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from custom_interfaces.srv import FindWall
from custom_interfaces.action import OdomRecord
from rclpy.action import ActionClient
import math

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        name_service = '/find_wall'
        self.client = self.create_client(FindWall, name_service)

        # Action client for navigation to pose
        self.action_track_client = ActionClient(
            self, OdomRecord, '/record_odom')

        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel',10)
        self.active = False
        self.current_state = None

        self.forward_speed = 0.1
        self.turning_speed = 0.0
        self.intial_orientation = True

        self.print_state = False

        self.req = FindWall.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.req)

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
            'front_left': list(range(deg_to_idx(0), deg_to_idx(60))),
            'side_left': list(range(deg_to_idx(60), deg_to_idx(120))), 
            'rear_left': list(range(deg_to_idx(120), deg_to_idx(180))), 
            'rear_right': list(range(deg_to_idx(180), deg_to_idx(240))), 
            'side_right': list(range(deg_to_idx(240), deg_to_idx(300))), 
            'front_right': list(range(deg_to_idx(300), deg_to_idx(360)))
        }

        
        min_distances = {key: float('inf') for key in sectors}
        for sector, ranges in sectors.items():
            #sector_ranges = [msg.ranges[i] for i in ranges if 0.0 < msg.ranges[i] < float('inf')]
            sector_ranges = [
                    msg.ranges[i] for i in ranges 
                    if not math.isnan(msg.ranges[i]) and msg.range_min < msg.ranges[i] < msg.range_max
                    ]
            if sector_ranges:
                min_distances[sector] = min(sector_ranges)

        front_indices = list(range(deg_to_idx(0), deg_to_idx(5)))+list(range(deg_to_idx(355), deg_to_idx(360)))
        valid_front = [msg.ranges[i] for i in front_indices if msg.range_min < msg.ranges[i] < msg.range_max]
        self.distance_to_front = sum(valid_front) / len(valid_front) if valid_front else float('inf')
        
        right_indices = list(range(deg_to_idx(265), deg_to_idx(276)))
        valid_right = [msg.ranges[i] for i in right_indices if not math.isnan(msg.ranges[i]) and msg.range_min < msg.ranges[i] < msg.range_max]
        self.distance_to_wall = min_distances['side_right']

        if self.active:
            if self.distance_to_front < 0.5:
                if self.current_state == 'Track new wall':
                    self.print_state = False
                else:
                    self.current_state = 'Track new wall'
                    self.print_state = True
                self.track_new_wall()
                return
            else:
                self.follow_wall()
                return

    def track_new_wall(self):
        msg_cmd = Twist()
        msg_cmd.angular.z = +0.6
        if self.print_state: self.get_logger().info(f'Track new wall! because dist = {self.distance_to_front:.2f}')
        self.publisher_cmd.publish(msg_cmd)
      
    def follow_wall(self):
        msg_cmd = Twist()
        if self.distance_to_wall > 0.3:
            msg_cmd.linear.x = self.forward_speed
            msg_cmd.angular.z = -0.1
            self.get_logger().info(f'Follow wall! approach wall. dist = {self.distance_to_wall:.2f}')
        elif self.distance_to_wall < 0.2:
            msg_cmd.linear.x = self.forward_speed * (self.distance_to_wall / 0.2)
            msg_cmd.angular.z = +0.1
            #self.get_logger().info(f'Follow wall! move away dist = {self.distance_to_wall:.2f}')
        else:
            msg_cmd.linear.x = self.forward_speed
            #self.get_logger().info(f'Follow wall! go forward dist = {self.distance_to_wall:.2f}')

        
        self.publisher_cmd.publish(msg_cmd)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[ACTION FEEDBACK] Total distance recorded so far: {feedback.current_total:.2f} m")

def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    
    # Declare the node constructor
    client = WallFollowerNode()
    
    # Run the send_request() method
    client.send_request()

    while rclpy.ok():
        # Spin once to check for a service response
        rclpy.spin_once(client)
        
        if client.future.done():
            try:
                # Check if a response from the service was received
                response = client.future.result()
            except Exception as e:
                # Log any exceptions
                client.get_logger().info(f'Service call failed: {e}')
            else:
                # Log the service response
                client.get_logger().info(f'Success: {response.wall_found}')
                if response.wall_found:
                    client.action_track_client.wait_for_server()
                    goal_msg = OdomRecord.Goal()
                    send_goal_future = client.action_track_client.send_goal_async(goal_msg,
                            feedback_callback=client.feedback_callback)
                    
                    rclpy.spin_until_future_complete(client, send_goal_future)
                    goal_handle = send_goal_future.result()
                    
                    if goal_handle.accepted:
                        client.get_logger().info('Odom recording goal accepted! Starting wall following loops.')
                        # Enable control callbacks safely
                        client.active = True
                    else:
                        client.get_logger().warn('Odom recording goal rejected by server!')

            break
        
    if rclpy.ok() and client.active:
        rclpy.spin(client)

    # Destroy the client node
    client.destroy_node()
    
    # Shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()