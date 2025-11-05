from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy

class LidarListener(Node):

    def __init__(self):
        super().__init__('lidar_listener')

        self.obstacle_publisher = self.create_publisher(String, '/obstacle_detected', 10)

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        front = msg.ranges[len(msg.ranges)//2]
        left = msg.ranges[len(msg.ranges)//4]
        right = msg.ranges[-len(msg.ranges)//4]

        self.get_logger().info(f"Front: {front:.2f} m, Left: {left:.2f} m, Right: {right:.2f} m")

        if front < 0.5:
            self.get_logger().warn("Obstacle detected ahead!")
            alert_msg = String()
            alert_msg.data = "Obstacle detected ahead!"
            self.obstacle_publisher.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

