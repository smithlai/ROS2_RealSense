

## PYTHON

ros2 pkg create --build-type ament_python realsense --dependencies rclpy


https://github.com/ros2/common_interfaces

simple_publisher.py
```py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Empty, 'mytopic', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        mag = Empty()
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_publisher = SimplePublisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroy the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
```py
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
```

mkdir launch
basic_launch.launch.py
```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='simple_publisher',
            output='screen'),

        Node(
            package='realsense',
            executable='simple_subscriber',
            output='screen'),
    ])
```


setup.py
```py
from setuptools import setup
import os
from glob import glob

package_name = 'realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))   # copy *.launch.py to install/realsense/share/<package_name>
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smith',
    maintainer_email='smith@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = '+package_name+'.simple_publisher:main',    # define python name
            'simple_subscriber = '+package_name+'.simple_subscriber:main',    # define python name
        ],
    },
)



```

`ros2 launch realsense basic_launch.launch.py`
you can test mytopic with 
`ros2 topic echo mytopic`