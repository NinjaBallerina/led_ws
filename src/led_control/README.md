# LED Control Package
The led_control package is a comprehensive educational tool designed to demonstrate the integration of ROS 2 with hardware components using a Raspberry Pi 5.
Developed by Farida and Dr. Alexandro Castellanos for his Applied Robotics class, this package makes it possible for users to control an LED through a GPIO button.
It uses the gpiozero library for GPIO management and ROS 2 Humble for coordinating the interaction logic between components.

- **LED Control**: Toggle an LED on and off with a simple button press.
- **Architecture**: Includes scripts for combined and separate publisher/subscriber setups, showing different ROS 2 design patterns.
- **ROS 2 Integration**: Shows node management and message passing within a ROS ecosystem.

## Requirements
- **ROS 2 Humble Hawksbill** installed on a Raspberry Pi 5.
- **Python 3** and the gpiozero library.
- Hardware setup includes a Raspberry Pi, an LED, a resistor, and a button.

## Hardware
1. **LED**: Connect the anode of the LED to GPIO 17 through a 220 ohm resistor to limit the current and prevent damage. 
2. **Button**: Connect one terminal of the button to GPIO 27 and the other terminal to the ground.

## Installation
1. **Clone the repository** into your ROS 2 workspace:
   `cd ~/led_ws`
   `git clone https://github.com/NinjaBallerina/led_ws.git`
2. **Build the package** using colcon build:
   `cd ~/led_ws`
   `colcon build`
3. **Source the environment** to include the newly built package:
   `source install/setup.bash`

## Running the Scripts
### Single Script for Combined Publisher and Subscriber
This is a simplified demonstration where one script manages both publishing and subscribing. Run this command:
   `ros2 run led_control led_button`
The script uses an integrated approach where button presses control the LED directly and publish its state, which is also handled internally.

### Separate Publisher and Subscriber Scripts
For a more complex system architecture that separates the publisher and subscriber roles, use the following commands:
1. **Run the publisher:**
   `ros2 run led_control led_publisher`
2. **Run the subscriber in a new terminal** (make sure this terminal is sourced):
   `source install/setup.bash`
   `ros2 run led_control led_subscriber`
The subscriber listens for toggle requests and controls the LED. It also gives feedback on the LED's state which the publisher uses to update its internal state.
