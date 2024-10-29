import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED

class LEDSubscriber(Node):
    def __init__(self):
        super().__init__('led_subscriber')
        self.led = LED(17)
        self.subscription = self.create_subscription(String, 'led_status', self.led_callback, 10)
        self.feedback_publisher = self.create_publisher(String, 'led_feedback', 10)

    def led_callback(self, msg):
        if msg.data == 'on':
            self.led.on()
            self.feedback_publisher.publish(String(data='on'))
            self.get_logger().info("Let there be light... and there was light!")
        elif msg.data == 'off':
            self.led.off()
            self.feedback_publisher.publish(String(data='off'))
            self.get_logger().info("Oops! Who stole the light?")

def main(args=None):
    rclpy.init(args=args)
    node = LEDSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
