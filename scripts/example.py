import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class Waypoint(Node):

    def __init__(self):
        super().__init__('nav2_waypoint')
        self.timer = self.create_timer(1, self.timer_callback)
        self.navigator = BasicNavigator()
        self.hri_route = [[6.049, 1.678],
                        [-0.233, 0.077],
                        [-0.394, -3.464],
                        [2.867, -3.499],
                        [6.592, -3.970]]
        self.total = 5
        self.time = 0
    
    def timer_callback(self):
        self.navigator.waitUntilNav2Active()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 6.049
        pose.pose.position.y = 1.678
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(pose)

        for (posx, posy) in hri_route:
            self.navigator.goToPose(pose)
            self.time += 1
            pose.pose.position.x = posx
            pose.pose.position.y = posy
            while not self.navigator.isTaskComplete():
                pass  
        feedback = self.navigator.getFeedback()
        self.timer.cancel()
        print(f"Time Elapsed: {self.time}")
        print(f"Feedback Timer: {feedback.navigation_time}")

def main(args=None):
    rclpy.init(args=args)
    wp = Waypoint()
    rclpy.spin(wp)
    wp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Updated Points: 6.049 1.678 -- -0.233 0.077 -- -0.394 -3.464 -- 2.867 -3.499 -- 6.592 -3.970