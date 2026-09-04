#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

class SimServoBridge(Node):
    def __init__(self):
        super().__init__('sim_servo_bridge')
        
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.pub_auto = self.create_publisher(Bool, '/auto_mode', 10)
        self.pub_challenge = self.create_publisher(String, '/set_challenge', 10)
        
        self.auto_mode = True
        self.prev_buttons = [0] * 15
        
        # Publish initial auto mode state
        self.create_timer(1.0, self.publish_auto_mode)
        
        self.get_logger().info('Sim Servo Bridge started (Auto Mode: True)')

    def publish_auto_mode(self):
        msg = Bool()
        msg.data = self.auto_mode
        self.pub_auto.publish(msg)

    def joy_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
            return

        # Start (Button 11 on some controllers) or Y (Button 4) to toggle auto mode
        toggle_pressed = (msg.buttons[11] == 1 and self.prev_buttons[11] == 0) or \
                         (msg.buttons[4] == 1 and self.prev_buttons[4] == 0)
        
        if toggle_pressed:
            self.auto_mode = not self.auto_mode
            self.publish_auto_mode()
            self.get_logger().info(f'Auto Mode toggled to: {self.auto_mode}')
            
        # LB (Button 6) / RB (Button 7) for challenge cycling
        if msg.buttons[6] == 1 and self.prev_buttons[6] == 0:
            chal = String()
            chal.data = 'PREV'
            self.pub_challenge.publish(chal)
            self.get_logger().info('Challenge: PREV')
            
        if msg.buttons[7] == 1 and self.prev_buttons[7] == 0:
            chal = String()
            chal.data = 'NEXT'
            self.pub_challenge.publish(chal)
            self.get_logger().info('Challenge: NEXT')
            
        self.prev_buttons = list(msg.buttons)

def main(args=None):
    rclpy.init(args=args)
    node = SimServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
