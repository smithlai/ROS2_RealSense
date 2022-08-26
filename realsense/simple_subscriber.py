import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Quaternion
from rclpy.qos import ReliabilityPolicy, QoSProfile

import serial
import math
import json
COM_PORT = '/dev/ttyACM0'    # 指定通訊埠名稱
BAUD_RATES = 115200    # 設定傳輸速率

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscriber = self.create_subscription(Quaternion, '/mytopic', self.mycallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber # prevent unused variable warning
        self.ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠

    def mycallback(self,msg):
        r, p, y = euler_from_quaternion(msg.x, msg.y, msg.z, msg.w)
        pitch = round(math.degrees(p))
        roll = round(math.degrees(r))
        yaw = round(math.degrees(y))
        enc = json.dumps({"p":pitch,"r":roll,"y":yaw}) + '\n'
        self.get_logger().info(enc)
        self.ser.write(enc.encode('utf-8'))
        # self.get_logger().info('Recving: "%s"' % enc)
        # if self.ser.in_waiting:          # 若收到序列資料…
        #     data_raw = self.ser.readline()  # 讀取一行
        #     data = data_raw.decode()   # 用預設的UTF-8解碼
        #     self.get_logger().info('Recving2: "%s"' % data)
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


                    
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    simplesubscriber = SimpleSubscriber()       
    rclpy.spin(simplesubscriber)
    simplesubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()