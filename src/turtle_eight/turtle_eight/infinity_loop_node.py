import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleInfinityNode(Node):
    """
    Węzeł ROS 2 do sterowania żółwiem w turtlesim po torze w kształcie ósemki.
    Działa w sposób ciągły i nieblokujący, zgodnie z dobrymi praktykami ROS 2.
    """

    def __init__(self):
        super().__init__('turtle_infinity_node')

        # === Konfiguracja parametrów ruchu ===
        # Zamiast "zaszywać" wartości w kodzie, definiujemy je w jednym miejscu.
        # W bardziej zaawansowanym ujęciu, można by je wczytywać z parametrów ROS 2.
        self.LINEAR_SPEED = 1.5  # Prędkość liniowa (m/s)
        self.ANGULAR_SPEED = 1.0  # Prędkość kątowa (rad/s)
        TIMER_PERIOD = 0.1  # Częstotliwość pracy pętli sterowania (10 Hz)
        
        # Obliczamy, ile kroków timera potrzeba na wykonanie jednego pełnego okręgu.
        # Czas na okrąg = 2 * PI / prędkość_kątowa
        # Kroki = Czas na okrąg / okres_timera
        self.LOOP_DURATION_STEPS = int((2 * math.pi / self.ANGULAR_SPEED) / TIMER_PERIOD)
        
        # === Inicjalizacja stanu węzła ===
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
        # Maszyna stanów: przechowuje informację, co aktualnie robi robot.
        self.state = 'TURN_POSITIVE'
        self.counter = 0  # Licznik kroków w danym stanie.
        self.twist_msg = Twist() # Obiekt wiadomości, tworzony raz i używany wielokrotnie.

        self.get_logger().info(f"Węzeł turtle_infinity_node uruchomiony. Rysowanie ósemki...")
        self.get_logger().info(f"Jeden łuk będzie trwał {self.LOOP_DURATION_STEPS * TIMER_PERIOD:.2f} s.")

    def timer_callback(self):
        """
        Główna pętla sterująca, wywoływana cyklicznie przez timer.
        Realizuje prostą maszynę stanów do rysowania ósemki.
        """
        # --- Logika maszyny stanów ---
        if self.state == 'TURN_POSITIVE':
            # Ustaw prędkości dla pierwszego łuku (obrót w lewo)
            self.twist_msg.linear.x = self.LINEAR_SPEED
            self.twist_msg.angular.z = self.ANGULAR_SPEED
            
            # Sprawdź, czy zakończyć ten stan
            if self.counter >= self.LOOP_DURATION_STEPS:
                self.state = 'TURN_NEGATIVE'  # Przejdź do następnego stanu
                self.counter = 0              # Zresetuj licznik
                self.get_logger().info('Zmiana kierunku na ujemny (w prawo).')

        elif self.state == 'TURN_NEGATIVE':
            # Ustaw prędkości dla drugiego łuku (obrót w prawo)
            self.twist_msg.linear.x = self.LINEAR_SPEED
            self.twist_msg.angular.z = -self.ANGULAR_SPEED
            
            # Sprawdź, czy zakończyć ten stan
            if self.counter >= self.LOOP_DURATION_STEPS:
                self.state = 'TURN_POSITIVE'  # Wróć do pierwszego stanu
                self.counter = 0              # Zresetuj licznik
                self.get_logger().info('Zmiana kierunku na dodatni (w lewo).')
        
        # --- Publikacja wiadomości i inkrementacja licznika ---
        self.publisher_.publish(self.twist_msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TurtleInfinityNode()
    try:
        # rclpy.spin() utrzymuje węzeł przy życiu i pozwala na wywoływanie
        # callbacków (np. od timera) w odpowiedzi na zdarzenia.
        # Program pozostaje w tej linii aż do zatrzymania (Ctrl+C).
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Obsługa przerwania z klawiatury (Ctrl+C)
        pass
    finally:
        # Ten blok wykona się zawsze przy zamykaniu, nawet po błędzie.
        # Zatrzymujemy żółwia przed zamknięciem.
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.get_logger().info('Zatrzymywanie żółwia i zamykanie węzła.')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
