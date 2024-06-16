import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DisplayLaserScan(Node):

    def __init__(self):
        # initialize node
        super().__init__("display_scan_node")
        
        # initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,
                                                 topic="/cmd_vel",
                                                 qos_profile=1)
        # initialize cmd_vel message variable
        self.twist_msg = Twist()

        # initialize scan subscriber
        self.scan_sub_qos = QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(msg_type=LaserScan,
                                                 topic="/laser/scan",
                                                 callback=self.scan_callback,
                                                 qos_profile=self.scan_sub_qos)
        
        # publish twist message with velocities
        self.twist_msg.linear.x = 0.50
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.75
        self.cmd_vel_pub.publish(self.twist_msg)

        return None

    def scan_callback(self, scan_msg):
        # read the range values from laserscan message
        ranges = scan_msg.ranges
        min_range = min(ranges)

        # print the range value
        self.get_logger().info("Range Min: %f" % (min_range))
        
        return None

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    display_scan_node = DisplayLaserScan()
    # spin the node
    rclpy.spin(display_scan_node)
    # destroy the node
    display_scan_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code