# Paczka ROS 2 - turtle_eight

Projekt w ROS 2, którego celem jest sterowanie żółwiem w `turtlesim`, by poruszał się on po torze w kształcie ósemki.

## Wynik końcowy

![Wynik działania programu](images/wynik.png)

## Wymagania

*   System operacyjny Ubuntu 22.04
*   ROS 2 Humble Hawksbill
*   Pakiet `turtlesim`

## Instalacja i Budowanie

1.  **Stwórz workspace:**
    ```bash
    mkdir -p turtle_eight_ws/src
    cd turtle_eight_ws/src
    ```

2.  **Sklonuj repozytorium do folderu `src`:**
    ```bash
    git clone https://github.com/your_name/your_repo.git
    ```

3.  **Wróć do głównego katalogu workspace'u i zbuduj projekt:**
    ```bash
    cd ..
    colcon build
    ```

## Uruchomienie

Do uruchomienia projektu potrzebne są **dwa osobne terminale**. W obu należy znajdować się w głównym folderze workspace (`turtle_eight_ws`).

#### **Terminal 1: Uruchomienie symulatora `turtlesim`**

```bash
# "Zasourcuj" środowisko ROS 2
source install/setup.bash

# Uruchom węzeł symulatora
ros2 run turtlesim turtlesim_node
```
Okno pozostaw je otwarte.

#### **Terminal 2: Uruchomienie węzła sterującego `turtle_eight`**

```bash
# "Zasourcuj" środowisko ROS 2 w tym terminalu również
source install/setup.bash

# Uruchom swój węzeł, który steruje żółwiem
ros2 run turtle_eight infinity_loop_node
```
Po wykonaniu tej komendy żółw w oknie symulatora powinien zacząć poruszać się po torze w kształcie ósemki.

---

## Sprawozdanie

### Cel Projektu

Celem było stworzenie węzła ROS 2, który steruje ruchem żółwia w symulatorze `turtlesim` po trajektorii w kształcie ósemki.

### Rozwiązanie początkowe
Google AI Studio zaproponowało najpierw koncepcję opierającą się na prostym, sekwencyjnym skrypcie Pythona. Mimo że był on funkcjonalny, jego architektura oparta na blokujących pętlach `while` i funkcji `time.sleep()` była sprzeczna z ideą ROS2, więc postanowiłem zmienić koncepcję.

### Finalne rozwiązanie
Rozwiązanie zostało zrealizowane jako **sterowany zdarzeniami węzeł ROS 2**. Jego architektura opiera się na **timerze** (`create_timer`), który cyklicznie wywołuje funkcję sterującą, oraz **maszynie stanów** (`self.state`), która zarządza logiką ruchu (skręt w lewo, skręt w prawo). Dzięki temu cała operacja jest napędzana przez wewnętrzny zegar ROS 2, a główna pętla programu (`rclpy.spin`) pozostaje otwarta na inne zdarzenia.

### Potencjalne Ulepszenia

1.  **Wykorzystanie Parametrów ROS 2**: Prędkości mogłyby być definiowane jako parametry węzła, co pozwoliłoby na ich zmianę z linii komend bez modyfikacji kodu.
2.  **Sterowanie w pętli zamkniętej**: Dla większej precyzji, węzeł mógłby subskrybować temat `/turtle1/pose`, aby na bieżąco korygować trajektorię na podstawie faktycznej pozycji żółwia.

## Licencja

Ten projekt jest udostępniany na licencji Apache License 2.0.
