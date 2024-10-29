import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Button

class LEDPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.publisher = self.create_publisher(String, 'led_status', 10)
        self.feedback_subscription = self.create_subscription(String, 'led_feedback', self.update_state, 10)
        self.button = Button(27, bounce_time=0.1)
        self.led_state = 'unknown' # The initial state of the LED is unknown.

        self.button.when_pressed = self.handle_press

    def update_state(self, msg):
        self.led_state = msg.data
        if self.led_state == 'on':
            self.get_logger().info("May the photons be with you.")
        else:
            self.get_logger().info("The light is off, and now we can all be ninjas.")

    def handle_press(self):
# Request a toggle based on the last known state:
        request_state = 'on' if self.led_state != 'on' else 'off'
        self.publisher.publish(String(data=request_state))

def main(args=None):
    rclpy.init(args=args)
    node = LEDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
