import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from std_msgs.msg import Empty

from rclpy.qos import ReliabilityPolicy, QoSProfile

class SimpleSubscriber(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('simple_subscriber')
        self.subscriber = self.create_subscription(Empty, '/mytopic', self.mycallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber # prevent unused variable warning

    def mycallback(self,msg):
        self.get_logger().info('Recving: "%s"' % msg)
                    
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simplesubscriber = SimpleSubscriber()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simplesubscriber)
    # Explicity destroy the node
    simplesubscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()