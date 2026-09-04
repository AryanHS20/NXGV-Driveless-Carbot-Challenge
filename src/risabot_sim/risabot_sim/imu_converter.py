#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32

class ImuConverter(Node):
    def __init__(self):
        super().__init__('imu_converter')
        
        self.sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10
        )
        self.pub_rpy = self.create_publisher(String, '/imu/rpy', 10)
        self.pub_pitch = self.create_publisher(Float32, '/imu/pitch', 10)
        
        self.get_logger().info('IMU Converter node started')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        
        # Quaternion to Euler angles (roll, pitch, yaw)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
            
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees for JSON
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Publish JSON string
        rpy_data = {
            "roll": roll_deg,
            "pitch": pitch_deg,
            "yaw": yaw_deg
        }
        str_msg = String()
        str_msg.data = json.dumps(rpy_data)
        self.pub_rpy.publish(str_msg)
        
        # Publish pitch Float32
        pitch_msg = Float32()
        pitch_msg.data = pitch_deg
        self.pub_pitch.publish(pitch_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
