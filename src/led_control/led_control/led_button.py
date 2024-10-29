import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED, Button
from gpiozero.pins.lgpio import LGPIOFactory
from signal import pause

class LEDButtonNode(Node):
    def __init__(self):
        super().__init__('led_button_node')
        self.led = LED(17)
        self.button = Button(27, bounce_time=0.1)
        self.publisher = self.create_publisher(String, 'led_status', 10)
        self.subscription = self.create_subscription(String, 'led_command', self.led_command_callback, 10)
        self.button.when_pressed = self.button_pressed

    def button_pressed(self):
        new_state = 'on' if not self.led.is_lit else 'off'
        self.toggle_led(new_state)
        self.publisher.publish(String(data=new_state))

    def toggle_led(self, command):
        if command == 'on':
            self.led.on()
            self.get_logger().info("Let there be light... and there was light!")
        else:
            self.led.off()
            self.get_logger().info("Oops! Who stole the light?")

    def led_command_callback(self, msg):
        self.toggle_led(msg.data)

def main(args=None):
    rclpy.init(args=args)
    led_button_node = LEDButtonNode()
    try:
        rclpy.spin(led_button_node)
    except KeyboardInterrupt:
        pass
    finally:
        led_button_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
