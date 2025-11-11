import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Emergency_Breaking(Node):

    def __init__(self):
        super().__init__('emergency_breaking')
        self.MIN_DEGREE =math.radians(-45)
        self.INCREMENT = math.radians(0.25)
        
        self.laser_sub = self.create_subscription(LaserScan,"/scan",self.lidar_callback,10)
        self.stop_pub = self.create_publisher(AckermannDriveStamped,"/safety_node",10)
        self.prev_closest = None


    
    def lidar_callback(self, msg: LaserScan):


        self.ranges = msg.ranges[360:720]
        if self.prev_closest is None:
            self.prev_closest = min(self.ranges)
            self.prev_time = self.get_clock().now().nanoseconds/1e9
            return
        self.current = min(self.ranges)
        self.idx = self.ranges.index(self.current)
        self.now = self.get_clock().now().nanoseconds/1e9
        self.v = min(0000.1,(self.current-self.prev_closest)/(self.now-self.prev_time))
        
        self.iTTC = self.current/self.v*math.cos(self.MIN_DEGREE+self.idx*self.INCREMENT)
        if self.iTTC<0.6:
            self.e_braking()
            self.e_brake_apply = True

    def e_braking(self):
        msg = AckermannDriveStamped()
        msg.drive._speed = 0.0
        self.stop_pub.publish(msg)



        
def main(args=None):
    rclpy.init(args=args)
    node = Emergency_Breaking()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()