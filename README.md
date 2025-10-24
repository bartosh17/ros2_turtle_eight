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

Jasne, oto sprawozdanie z realizacji projektu na podstawie dostarczonego kodu, w formacie Markdown.

### **Sprawozdanie z Realizacji Projektu: Sterowanie Żółwiem w ROS2**

Poniższe sprawozdanie opisuje realizację projektu polegającego na stworzeniu węzła ROS2 do sterowania symulatorem `turtlesim`, tak aby wirtualny żółw rysował okręgi.

---

### **Cel Projektu**

Głównym celem projektu było zaimplementowanie w ROS2 węzła w języku Python, który publikuje komunikaty typu `geometry_msgs/msg/Twist` do tematu `/turtle1/cmd_vel`. Węzeł miał za zadanie sterować żółwiem w symulatorze `turtlesim` w taki sposób, aby narysował on okrąg o zadanym promieniu i w określonym kierunku (zgodnie z ruchem wskazówek zegara lub przeciwnie).

---

### **Koncepcja**

Koncepcja opierała się na stworzeniu klasy `TurtleCircleNode` dziedziczącej po `rclpy.node.Node`. Węzeł ten miał zawierać:
1.  **Publishera**: Do wysyłania poleceń prędkości (`Twist`) do symulatora.
2.  **Metodę `draw_circle`**: Funkcję realizującą logikę rysowania okręgu, przyjmującą jako argumenty promień oraz kierunek obrotu.
3.  **Logikę sterowania**: Obliczenie i ustawienie stałej prędkości liniowej (`linear.x`) i kątowej (`angular.z`), które w połączeniu powodują ruch po okręgu. Czas trwania ruchu został obliczony na podstawie pełnego obrotu (2π radianów).

---

### **Rozwiązanie Początkowe (Zaimplementowane w kodzie)**

Kluczowym elementem tej implementacji jest pętla `while` wewnątrz metody `draw_circle`, która w połączeniu z `time.sleep(0.01)` blokuje główny wątek programu na czas rysowania okręgu. Mimo że rozwiązanie to jest funkcjonalne i osiąga zamierzony cel, jego architektura jest niezgodna z dobrymi praktykami i filozofią ROS2, która promuje architekturę sterowaną zdarzeniami i unikanie blokujących operacji.


### **Finalne Rozwiązanie (Propozycja zgodna z ROS2)**

Aby dostosować kod do standardów ROS2, należałoby unikać blokujących pętli `while` i `time.sleep()`. Lepszym podejściem byłoby wykorzystanie **timera** (`create_timer`), który cyklicznie wywołuje funkcję zwrotną (callback) publikującą komunikaty `Twist`.

**Koncepcja finalnego rozwiązania:**
1.  W metodzie `draw_circle` zamiast pętli `while`, tworzony jest timer, który co określony interwał (np. 0.01s) wywołuje funkcję `timer_callback`.
2.  Funkcja `timer_callback` publikuje komunikat `Twist` i sprawdza, czy upłynął już czas potrzebny na narysowanie okręgu.
3.  Po zakończeniu rysowania, timer jest anulowany (`timer.cancel()`), a do symulatora wysyłana jest komenda zatrzymania żółwia.
4.  Całość jest zarządzana przez `rclpy.spin(node)`, co pozwala węzłowi na obsługę zdarzeń w sposób nieblokujący.

Takie podejście sprawia, że węzeł pozostaje responsywny i może w przyszłości obsługiwać inne zadania równocześnie (np. subskrybować inne tematy lub odpowiadać na usługi).

---

### **Potencjalne Ulepszenia**
1.  **Refaktoryzacja z użyciem timera**: Najważniejszym ulepszeniem jest przepisanie logiki rysowania z użyciem `create_timer`, aby uniknąć blokowania wątku.
2.  **Wykorzystanie parametrów ROS2**: Promień, kierunek rysowania i prędkości mogłyby być zdefiniowane jako parametry węzła, co pozwoliłoby na ich łatwą konfigurację z zewnątrz (np. z pliku `yaml` lub z linii komend) bez potrzeby modyfikacji kodu.
3.  **Implementacja jako usługa (Service)**: Zamiast rysować okręgi sekwencyjnie przy starcie, można by stworzyć usługę ROS2, która przyjmowałaby żądanie narysowania okręgu z określonymi parametrami. To uczyniłoby węzeł bardziej modułowym i użytecznym w większych systemach.
4.  **Użycie pętli `Rate`**: Zamiast `time.sleep()`, można wykorzystać `node.create_rate()`, która zapewnia bardziej stabilną częstotliwość pętli w kontekście ROS2.

## Licencja

Ten projekt jest udostępniany na licencji Apache License 2.0.
