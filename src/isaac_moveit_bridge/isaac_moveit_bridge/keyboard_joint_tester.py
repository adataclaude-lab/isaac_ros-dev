#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os
import math
import sys
import tty
import termios

class KeyboardJointPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_joint_publisher')
        
        self.joint_names = [
            'j1_Joint_L', 'j2_Joint_L',
            'j3_Joint_L', 'j4_Joint_L',
            'j5_Joint_L', 'j6_Joint_L',
            'j6_fingerA_Joint_L', 'j6_fingerB_Joint_L'
        ]
        
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        
        # Initial positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.active_joint_index = 0
        self.angle_step = 0.1  # Radians (~5.7 degrees)

        self.get_logger().info("🚀 Keyboard Joint Publisher started.")
        self.get_logger().info("Publishing to /joint_command")

    def get_key(self, settings):
        """Read a single keypress from the terminal."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_states(self):
        """Publish the current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher_.publish(msg)

    def print_ui(self):
        """Clear the screen and print the current state."""
        # Clear the terminal screen
        os.system('clear') 
        
        # Print the header and controls
        print("--- Keyboard Joint Publisher ---")
        print("Controls: [W/S] Change Angle | [A/D] Select Joint | [Q] Quit\n")
        
        # Loop through all joints
        for i, (name, pos_rad) in enumerate(zip(self.joint_names, self.joint_positions)):
            
            # Check if this is the currently selected joint
            prefix = ">> " if i == self.active_joint_index else "   "
            
            # Convert the radian position (pos_rad) to degrees
            deg = math.degrees(pos_rad)
            
            # Print the joint name and its position in degrees
            print(f"{prefix}{name:<15}: {deg:+.2f} degrees")

        print("\n--- Publishing to /joint_command ---")

    def run_loop(self):
        """Main loop to read keys and publish."""
        # Save current terminal settings
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                self.print_ui()
                self.publish_states()
                
                key = self.get_key(settings)
                
                if key == 'q' or key == '\x03':  # q or Ctrl+C
                    break
                elif key == 'w':
                    self.joint_positions[self.active_joint_index] += self.angle_step
                elif key == 's':
                    self.joint_positions[self.active_joint_index] -= self.angle_step
                elif key == 'a':
                    self.active_joint_index = (self.active_joint_index - 1) % len(self.joint_names)
                elif key == 'd':
                    self.active_joint_index = (self.active_joint_index + 1) % len(self.joint_names)
                
                # Clamp values between -pi and pi
                pos = self.joint_positions[self.active_joint_index]
                self.joint_positions[self.active_joint_index] = max(-math.pi, min(math.pi, pos))

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_publisher = KeyboardJointPublisher()
    
    try:
        # This line was failing because the run_loop method was missing
        keyboard_publisher.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()