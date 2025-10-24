import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleCircleNode(Node):

    def __init__(self):
        super().__init__('turtle_circle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Węzeł TurtleCircle został uruchomiony.')

    def draw_circle(self, radius, clockwise):
        """
        Rysuje okrąg o zadanym promieniu i kierunku.
        """
        if clockwise:
            self.get_logger().info(f"Rysowanie koła o promieniu {radius} zgodnie z ruchem wskazówek zegara.")
        else:
            self.get_logger().info(f"Rysowanie koła o promieniu {radius} przeciwnie do ruchu wskazówek zegara.")

        # Używamy komunikatu typu Twist do sterowania żółwiem
        vel_msg = Twist()

        # Prędkość liniowa jest stała
        vel_msg.linear.x = radius
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        # Prędkość kątowa zależy od kierunku
        if clockwise:
            vel_msg.angular.z = -1.0
        else:
            vel_msg.angular.z = 1.0

        # Czas potrzebny na narysowanie pełnego koła
        duration = 2 * math.pi
        start_time = self.get_clock().now()

        # Pętla publikująca polecenia prędkości
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(vel_msg)
            time.sleep(0.01)

        # Zatrzymaj żółwia po narysowaniu koła
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)
        self.get_logger().info('Zakończono rysowanie koła.')
        time.sleep(1) # Chwila przerwy

def main(args=None):
    rclpy.init(args=args)

    node = TurtleCircleNode()

    # Rysowanie koła zgodnie z ruchem wskazówek zegara
    node.draw_circle(1.0, True)

    # Rysowanie koła przeciwnie do ruchu wskazówek zegara
    node.draw_circle(1.0, False)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()