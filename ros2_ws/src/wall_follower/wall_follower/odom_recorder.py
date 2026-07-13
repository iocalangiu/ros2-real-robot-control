import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from math import sqrt, atan2
from custom_interfaces.action import OdomRecord
from geometry_msgs.msg import Point
import time
import rclpy.duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class OdomActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        self.group = ReentrantCallbackGroup()
        # Action server to accept goals
        self.action_server = ActionServer(
            self, OdomRecord, 'record_odom', self.execute_callback,callback_group=self.group)

        # Subscription to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.group)

        self.last_odom = Point()
        self.first_odom = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        
        x = position.x
        y = position.y
        self.last_odom.x = x
        self.last_odom.y = y
        q = msg.pose.pose.orientation
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        
        self.last_odom.z = atan2(siny_cosp, cosy_cosp)

    async def execute_callback(self, goal_handle):
        self.first_odom = Point()
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y
        
        self.total_distance = 0.0
        self.last_x = self.first_odom.x
        self.last_y = self.first_odom.y

        self.odom_record = []
        result = OdomRecord.Result()
        feedback_msg = OdomRecord.Feedback()

        self.get_logger().info(f"Starting tracking. First Odom: x={self.first_odom.x:.2f}, y={self.first_odom.y:.2f}")
        has_left_start_zone = False

        while goal_handle.is_active:
            if not goal_handle.is_active:
                self.get_logger().info('Goal is no longer active.')
                goal_handle.abort()
                result.list_of_odoms = self.odom_record
                return result

            current_snap = Point()
            current_snap.x = self.last_odom.x
            current_snap.y = self.last_odom.y
            current_snap.z = self.last_odom.z

            segment_distance = sqrt((current_snap.x - self.last_x)**2 + (current_snap.y - self.last_y)**2)
            self.total_distance += segment_distance
            
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)

            distance_to_start = sqrt((self.first_odom.x - current_snap.x)**2 + (self.first_odom.y - current_snap.y)**2)
            
            self.get_logger().info(f"Recorded point. Total Distance: {self.total_distance:.2f} m and distance to start: {distance_to_start:.2f}")

            self.last_x = current_snap.x
            self.last_y = current_snap.y
            
            self.odom_record.append(current_snap)

            
            
            if not has_left_start_zone and distance_to_start > 1:
                has_left_start_zone = True
                self.get_logger().info("Robot has left the starting zone. Monitoring for loop closure...")

            if has_left_start_zone and distance_to_start < 0.4:
                self.get_logger().info(f"Loop completed! Returned to start. Distance to start: {distance_to_start:.2f} m")
                break
            time.sleep(1.0)


        # 3. Mark the goal as succeeded and return the results
        goal_handle.succeed()
        
        # Assign your collected array to the action file's defined list field
        result.list_of_odoms = self.odom_record
        self.get_logger().info('Finished recording odometry paths. Returning result.')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = OdomActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()  
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
