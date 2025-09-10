#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class ClockNode(Node):
    """
    Publishes /clock at a fixed dt (seconds). Use with 'use_sim_time:=true' elsewhere.
    """
    def __init__(self):
        super().__init__('sim_clock')
        self.declare_parameter('dt', 0.05)          # 20 Hz default
        self.declare_parameter('start_time', 0.0)   # seconds
        self.declare_parameter('paused', False)

        self.dt = float(self.get_parameter('dt').value)
        self.sim_time = float(self.get_parameter('start_time').value)
        self.paused = bool(self.get_parameter('paused').value)

        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"sim_clock running: dt={self.dt:.3f}s start={self.sim_time:.3f}s paused={self.paused}")

    def _tick(self):
        if not self.paused:
            self.sim_time += self.dt
        sec = int(self.sim_time)
        nsec = int((self.sim_time - sec) * 1e9)
        self.pub.publish(Clock(clock=Time(sec=sec, nanosec=nsec)))

def main():
    rclpy.init()
    node = ClockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
