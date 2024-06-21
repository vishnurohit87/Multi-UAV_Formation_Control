import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from helpers import Waypoint


class GPSSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            String,
            'target_gps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.gps_data = Waypoint(0, 0)

    def listener_callback(self, msg):
        # self.get_logger().info('Received: "%s"' % msg.data)
        self.gps_data = msg.data.split(", ")
        # print(f"GPS Data: {self.gps_data}")
        self.gps_data = Waypoint(float(self.gps_data[0]), float(self.gps_data[1]))

    def get_gps_data(self):
        return self.gps_data

def ros_spin(node):
    rclpy.spin(node)

def main(args=None): # For testing purposes
    rclpy.init(args=args)

    minimal_subscriber = GPSSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()