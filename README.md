# Smart Traffic Light System
**COE411 – Embedded and Cyber Physical Systems | Milestone 2**

| Name | ID |
|---|---|
| Youssef Hussien | 94899 |
| Youssef Abdellatife | 94622 |
| Seif Hussien | 97215 |

---

## Project Overview

An intelligent intersection controller built on the ESP32 using FreeRTOS.
Simulated entirely in Wokwi via VS Code + PlatformIO. No physical hardware required.

### Features
- Standard Green → Yellow → Red traffic cycle
- Pedestrian crossing via push button (forces red immediately)
- Vehicle detection via HC-SR04 ultrasonic sensor (extends green by 5 s, max 3×)
- PWM buzzer alerts via LEDC driver
- FreeRTOS Rate-Monotonic scheduling with binary semaphores and queue

---

## GPIO Pin Mapping

| Component | ESP32 Pin | Purpose |
|---|---|---|
| RGB LED – Red | GPIO 25 | Red traffic light |
| RGB LED – Yellow | GPIO 26 | Yellow traffic light |
| RGB LED – Green | GPIO 27 | Green traffic light |
| Buzzer | GPIO 32 | PWM alert (LEDC) |
| Push Button | GPIO 19 | Pedestrian crossing request |
| HC-SR04 TRIG | GPIO 5 | Ultrasonic pulse trigger |
| HC-SR04 ECHO | GPIO 18 | Echo return |

---

## FreeRTOS Task Summary

| Task | Priority | Period | Purpose |
|---|---|---|---|
| Pedestrian Task | 4 (highest) | Aperiodic | Button press → force red |
| Traffic Cycle Task | 3 | 1000–10000 ms | Green/Yellow/Red sequence |
| Ultrasonic Task | 2 | 100 ms | Distance sensing |
| Buzzer Task | 1 (lowest) | Queue-driven | PWM tone generation |

---

## Project Structure

```
smart_traffic_light/
├── src/
│   └── main.c          ← All firmware code (tasks, ISR, drivers)
├── diagram.json        ← Wokwi circuit diagram
├── wokwi.toml          ← Wokwi + PlatformIO integration config
├── platformio.ini      ← PlatformIO build configuration
└── README.md
```

---

## How to Run (Wokwi Simulation)

### Prerequisites
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
- [Wokwi for VS Code extension](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode)
- Wokwi licence (free for students)

### Steps

1. **Clone / open the project folder** in VS Code.

2. **Build the firmware:**
   ```
   PlatformIO: Build   (Ctrl+Alt+B)
   ```

3. **Start the Wokwi simulation:**
   - Open `diagram.json`
   - Click the **▶ Start Simulation** button in VS Code (or press `F1` → *Wokwi: Start Simulator*)

4. **Interact with the simulation:**
   - Watch the RGB LED cycle Green → Yellow → Red.
   - Click the **green push button** to trigger a pedestrian crossing (LED goes red, buzzer beeps).
   - In the Wokwi panel, set the **HC-SR04 distance slider** to < 20 cm while green is active to trigger a green extension.

5. **View serial output** in the PlatformIO Serial Monitor for real-time task logs.

---

## Timing Constants (adjustable in `main.c`)

| Constant | Default | Description |
|---|---|---|
| `GREEN_PHASE_MS` | 5000 ms | Normal green duration |
| `YELLOW_PHASE_MS` | 2000 ms | Yellow transition |
| `RED_PHASE_MS` | 4000 ms | Normal red duration |
| `PEDESTRIAN_CROSSING_MS` | 5000 ms | Extra red for safe crossing |
| `GREEN_EXTENSION_MS` | 5000 ms | Green extension per detection |
| `MAX_EXTENSIONS` | 3 | Max consecutive extensions |
| `ULTRA_DETECT_CM` | 20 cm | Detection threshold |

---

## Inter-Task Communication

```
[Pedestrian Task] ──xSemPedestrian──►[Traffic Cycle Task]
[Ultrasonic Task] ──xSemExtension───►[Traffic Cycle Task]
[Pedestrian Task] ──xQueueBuzzer────►[Buzzer Task]
[Traffic Cycle Task]──xQueueBuzzer──►[Buzzer Task]
```
