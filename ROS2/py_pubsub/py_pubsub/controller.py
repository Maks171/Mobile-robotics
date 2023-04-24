import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.publisher_odom = self.create_publisher(Odometry,"/odom",10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_odom,
            10)
        self.subscription  # prevent unused variable warning

    def listener_odom(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation

        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        #self.get_logger().info('%f' %position_x)
        #self.get_logger().info('%f' %position_y)
        self.get_logger().info('%f' %yaw)
        



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalSubscriber()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
