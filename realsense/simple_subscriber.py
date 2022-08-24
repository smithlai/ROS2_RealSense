import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from rclpy.qos import ReliabilityPolicy, QoSProfile

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscriber = self.create_subscription(Empty, '/mytopic', self.mycallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber # prevent unused variable warning

    def mycallback(self,msg):
        self.get_logger().info('Recving: "%s"' % msg)
                    
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    simplesubscriber = SimpleSubscriber()       
    rclpy.spin(simplesubscriber)
    simplesubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()