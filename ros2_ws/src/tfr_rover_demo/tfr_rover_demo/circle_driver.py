# tfr_rover_demo/circle_driver.py
# Publishes geometry_msgs/Twist on /cmd_vel to drive in a gentle circle.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')

        # Parameters (can be overridden at runtime)
        self.declare_parameter('linear', 0.3)   # m/s
        self.declare_parameter('angular', 0.3)  # rad/s

        # Publisher & timer
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self._tick)  # 2 Hz

        # Allow live param updates if you 'ros2 param set ...'
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info('circle_driver started (publishing /cmd_vel at 2 Hz)')

    def _on_params(self, params):
        for p in params:
            if p.name in ('linear', 'angular') and (not isinstance(p.value, (int, float))):
                from rcl_interfaces.msg import SetParametersResult
                return SetParametersResult(successful=False, reason=f'Param {p.name} must be numeric')
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _tick(self):
        lin = float(self.get_parameter('linear').value)
        ang = float(self.get_parameter('angular').value)

        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()