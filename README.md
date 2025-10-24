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

## Sprawozdanie Techniczne

### Cel Projektu

Celem było stworzenie węzła ROS 2, który steruje ruchem żółwia w symulatorze `turtlesim` po trajektorii w kształcie ósemki, zaimplementowanego zgodnie z najlepszymi praktykami programowania w ROS 2.

### Opis Zaimplementowanego Rozwiązania

Rozwiązanie zostało zrealizowane jako **sterowany zdarzeniami, nieblokujący węzeł ROS 2**. Jego architektura opiera się na **timerze** (`create_timer`), który cyklicznie wywołuje funkcję sterującą, oraz **maszynie stanów** (`self.state`), która zarządza logiką ruchu (skręt w lewo, skręt w prawo). Dzięki temu cała operacja jest napędzana przez wewnętrzny zegar ROS 2, a główna pętla programu (`rclpy.spin`) pozostaje otwarta na inne zdarzenia.

### Ewolucja Rozwiązania
Warto zaznaczyć, że pierwotna koncepcja rozwiązania, sugerowana przez narzędzia AI, opierała się na prostym, sekwencyjnym skrypcie Pythona. Mimo że był on funkcjonalny, jego architektura oparta na blokujących pętlach `while` i funkcji `time.sleep()` była sprzeczna z filozofią ROS 2. Taki skrypt nie jest w stanie działać w sposób ciągły i reaktywny, co jest kluczowe w robotyce. Z tego powodu podjęto świadomą decyzję o porzuceniu tego podejścia na rzecz obecnej, znacznie bardziej solidnej implementacji.

### Zalety obecnego podejścia

| Cecha | Podejście Zdarzeniowe (Zaimplementowane) | Podejście Blokujące (Prosty Skrypt) |
| :--- | :--- | :--- |
| **Reaktywność** | **Wysoka.** Węzeł jest zawsze gotowy na inne zdarzenia. | **Brak.** Węzeł jest "zamrożony" na czas wykonywania pętli. |
| **Elastyczność** | **Duża.** Łatwa rozbudowa o nowe stany/zachowania. | **Niska.** Każda zmiana wymaga przebudowy logiki. |
| **Zgodność z ROS**| **Pełna.** Idiomatyczny sposób pisania węzłów w ROS 2. | **Minimalna.** Ignoruje architekturę i paradygmaty ROS. |

### Potencjalne Ulepszenia

1.  **Wykorzystanie Parametrów ROS 2**: Prędkości mogłyby być definiowane jako parametry węzła, co pozwoliłoby na ich zmianę z linii komend bez modyfikacji kodu.
2.  **Sterowanie w pętli zamkniętej**: Dla większej precyzji, węzeł mógłby subskrybować temat `/turtle1/pose`, aby na bieżąco korygować trajektorię na podstawie faktycznej pozycji żółwia.

## Licencja

Ten projekt jest udostępniany na licencji Apache License 2.0.
