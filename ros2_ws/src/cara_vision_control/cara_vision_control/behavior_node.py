#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CaraBehaviorNode(Node):
    def __init__(self):
        super().__init__('cara_behavior_node')

        self.sub = self.create_subscription(
            String,
            '/cara/emotion_state',
            self.emotion_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/cara/behavior_cmd',
            10
        )

        self.latest_label = 'neutral'
        self.latest_conf = 0.0
        self.head_angle = 90.0

        #self.timer = self.create_timer(0.15, self.publish_behavior) too fast
        self.timer = self.create_timer(0.4, self.publish_behavior)
    def emotion_callback(self, msg):
        try:
            label, conf = msg.data.split(',')
            self.latest_label = label.strip().lower()
            self.latest_conf = float(conf.strip())
        except:
            pass

    def map_emotion(self):
        if self.latest_conf < 0.5:
            return 90, 0

        if self.latest_label == 'sad':
    	    return 75, 1 if self.latest_conf > 0.7 else 0
        elif self.latest_label == 'happy':
            return 105, 0
        elif self.latest_label == 'surprise':
            return 115, 0
        else:
            return 90, 0

    def publish_behavior(self):
        target, blink = self.map_emotion()

        # smoothing
        self.head_angle = 0.9 * self.head_angle + 0.2 * target

        msg = String()
        msg.data = f'HEAD:{int(self.head_angle)},BLINK:{blink}'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CaraBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
