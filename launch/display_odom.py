import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DisplayOdometry(Node):

    def __init__(self):
        # initialize node
        super().__init__("display_odom_node")
        
        # initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,
                                                 topic="/cmd_vel",
                                                 qos_profile=1)
        # initialize cmd_vel message variable
        self.twist_msg = Twist()

        # initialize odom subscriber
        self.odom_sub_qos = QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)
        self.odom_sub = self.create_subscription(msg_type=Odometry,
                                                 topic="/odom",
                                                 callback=self.odom_callback,
                                                 qos_profile=self.odom_sub_qos)
        
        # publish twist message with velocities
        self.twist_msg.linear.x = 0.50
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.75
        self.cmd_vel_pub.publish(self.twist_msg)

        return None

    def odom_callback(self, odom_msg):
        # read the position values from odometry message
        position = odom_msg.pose.pose.position
        pos_x = position.x
        pos_y = position.y

        # print the positions of x and y
        self.get_logger().info("PosX: %f, PosY: %f" % (pos_x, pos_y))
        
        return None

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    display_odom_node = DisplayOdometry()
    # spin the node
    rclpy.spin(display_odom_node)
    # destroy the node
    display_odom_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code