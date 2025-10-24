import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleInfinityNode(Node):

    def __init__(self):
        super().__init__('turtle_infinity_node')

        self.LINEAR_SPEED = 1.5
        self.ANGULAR_SPEED = 1.0
        TIMER_PERIOD = 0.1
        
        self.LOOP_DURATION_STEPS = int((2 * math.pi / self.ANGULAR_SPEED) / TIMER_PERIOD)
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
        self.state = 'TURN_POSITIVE'
        self.counter = 0
        self.twist_msg = Twist()

        self.get_logger().info(f"Rysowanie ósemki...")

    def timer_callback(self):

        if self.state == 'TURN_POSITIVE':
            self.twist_msg.linear.x = self.LINEAR_SPEED
            self.twist_msg.angular.z = self.ANGULAR_SPEED
            
            if self.counter >= self.LOOP_DURATION_STEPS:
                self.state = 'TURN_NEGATIVE'
                self.counter = 0
                self.get_logger().info('Zmiana kierunku na ujemny (w prawo).')

        elif self.state == 'TURN_NEGATIVE':
            
            self.twist_msg.linear.x = self.LINEAR_SPEED
            self.twist_msg.angular.z = -self.ANGULAR_SPEED
            
            if self.counter >= self.LOOP_DURATION_STEPS:
                self.state = 'TURN_POSITIVE'
                self.counter = 0
                self.get_logger().info('Zmiana kierunku na dodatni (w lewo).')
        
        self.publisher_.publish(self.twist_msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TurtleInfinityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.get_logger().info('Zatrzymywanie żółwia i zamykanie węzła.')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
