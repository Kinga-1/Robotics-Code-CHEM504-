# Robotics-Code-CHEM504-
## Automation of the Traffic Light Redox Reaction

### Aim: Design and implement an autonomous system to automate the ‘Traffic Light’ redox reaction

**Course:** CHEM504 &nbsp;|&nbsp; **Language:** Python 3 &nbsp;|&nbsp; **License:** MIT

---

## Overview

This project implements a fully autonomous robotic system to run the **Traffic Light redox reaction** without human intervention. The system uses a UR5 robotic arm to physically handle the reaction vial, an IKA RCT Digital stirring plate to control oxygenation, and two Logitech cameras to monitor colour transitions in real time, with a custom-made camera stand made in autodesk fusion. A Tkinter-based dashboard provides live visualisation of reaction kinetics, an event log, and an emergency stop control.

The reaction cycles through **green --> red --> yellow** as the indigo carmine is reduced, before circling back through red to green upon oxidation. The robot detects each colour transition using HSV-based computer vision and responds autonomously - moving the vial to or from the stirring plate at the correct moment across two complete oscillation cycles.

---
## The reaction 
The sample solution is made up manually inside a 10 mL sample vial, with the following proportions added in the following order: 
- 5 mL sodium hydroxide (NaOH), 
-  1.5 mL glucose, 
- 15 drops of indigo carmine via pipette

And sealed with a lid to limit the oxygen availability. 

## Hardware

| Component | Role |
|---|---|
| **UR5 Robotic Arm** (Universal Robots) | Vial handling - pick, place, transport |
| **Robotiq Gripper** (attached to UR5) | Gripping and releasing the reaction vial |
| **IKA RCT Digital Stirring Plate** | Controlled stirring to reoxygenate the solution |
| **Logitech Camera 1** (`index 0`) | Filming camera - monitors vial at the white-background filming position |
| **Logitech Camera 2** (`index 2`) | Plate camera - monitors vial while on the stirring plate |

---

## Repository Structure

```
Robotics-Code-CHEM504-/
├── Code/
│   ├── experiment.py               # Entry point: hardware setup, threading, UI launch
│   ├── traffic_light_experiment.py # Experiment sequence logic (main workflow)
│   ├── ui.py                       # Tkinter dashboard with live kinetics graphs
│   ├── colour_monitor.py           # Threaded colour state monitor per camera
│   ├── kinetics_live.py            # HSV colour detection, kinetics engine, graph data
│   ├── gripper_helpers.py          # Open/close gripper wrappers
│   ├── positions_list.py           # UR5 joint-space positions
│   └── utils/
│       ├── UR_Functions.py         # UR5 communication interface
│       ├── ika_handler.py          # IKA stirring plate serial interface
│       └── robotiq_gripper.py      # Robotiq gripper driver
├── LICENSE
└── README.md
```

---

## Dependencies

```
pip install numpy opencv-python scipy math cv2 time threading collections datetime tkinter os socket functools
```

The following internal utilities are also required (included under `utils/`):

- `UR_Functions` — TCP socket interface to the UR5 controller
- `ika_handler` — Serial interface to the IKA RCT Digital plate
- `robotiq_gripper` — Driver for the Robotiq gripper
almongst others 

---

## How It Works

### Operation Modes

On launch, a dialog prompts the user to select one of two positioning modes:

- **Aruco Mode** — The UR5 moves to a scan position and uses ArUco marker detection (via `ArucoPlacementSystem`) to locate the vial rack hole and stirring plate centre. All subsequent TCP poses are computed dynamically from the detected coordinates.
- **Coordinate Mode** — The arm moves using pre-recorded joint-space positions stored in `positions_list.py`. This mode requires no visual calibration but assumes fixed lab geometry.

### Experiment Sequence

Once a mode is selected, the system runs the following sequence autonomously:

1. **Pick vial** - UR5 moves to the vial rack, closes the gripper, and lifts the vial.
2. **Move to filming position** - Vial is held in front of the filming camera (white background) for colour monitoring.
3. **Wait: GREEN → RED → YELLOW** - `filmMonitor` blocks until both transitions are detected. A frame is captured at YELLOW.
4. **Cycle 1 — Place on stirring plate** - Vial is moved to the IKA plate; stirring starts at 1500 RPM. Kinetics source switches to `plateMonitor`.
5. **Wait: YELLOW → RED** - Stirring stops when RED is detected.
6. **Return to filming position** - Kinetics source switches back to `filmMonitor`. A frame is captured.
7. **Cycle 2 — Place on stirring plate again** - Stirring resumes.
8. **Wait: RED → GREEN** - Stirring stops when GREEN is detected.
9. **Return to filming position** - A final frame is captured.
10. **Return vial to rack** - UR5 places the vial back and returns to the home position.

### Colour Detection

Colour state is determined in `kinetics_live.py` using OpenCV HSV masking over the centre 50% ROI of each camera frame. Per-colour pixel counts determine the dominant state; detections below a minimum pixel threshold are classified as `none` to reject noise.

HSV ranges used:

| Colour | Hue range | Notes |
|---|---|---|
| Green | 35 – 85 | Broad to accommodate methylene blue in reduced state |
| Red | 0 – 10 and 160 – 180 | Wraps around the HSV hue circle |
| Yellow | 20 – 35 | Intermediate oxidised state |

### Kinetics Engine

`KineticsEngine` in `kinetics_live.py` logs every colour transition (debounced at 0.4 s) and derives:

- **Induction period** - time to the first real transition
- **Mean cycle period** - average time between recurrences of the same colour
- **Phase durations** - mean time spent in each colour state
- **Oscillation count** - number of complete green → red → yellow cycles
- **Anomaly detection** - flags transitions that deviate more than 2× or less than 0.5× from their historical mean

The `LiveKineticsRunner` class runs separate processing threads for each camera and feeds the active source into the kinetics engine. The active source switches mid-experiment (filming → plate → filming) without interrupting data collection.

### Dashboard

The Tkinter UI (`ui.py`) displays:

- Live frames from both cameras with the current colour state overlaid
- A concentration proxy graph (normalised hue over time, colour-coded by state)
- A reaction rate graph (|ΔH/Δt| over time)
- A transition event log with wall-clock timestamps
- An **ABORT** button that triggers `safe_stop()` at any point

### Emergency Stop

`safe_stop()` in `experiment.py` can be triggered by the abort button, window close, `KeyboardInterrupt`, `ColourTimeoutError`, or any unhandled exception. It:

1. Immediately releases both `ColourMonitor` instances, unblocking any waiting transition
2. Sends a `stopj(2.0)` command directly to the UR5 controller socket
3. Attempts to return the vial to the rack and move the arm to the home position
4. Stops stirring
5. Freezes the UI in a static state for post-experiment review

---

## Running the Experiment

### Prerequisites

1. UR5 controller is powered on and set to **Remote Control** mode.
2. IKA stirring plate is connected via USB/serial.
3. Both Logitech cameras are connected (indices `0` and `2`).
4. UR5 controller is reachable at `192.168.0.2` (configurable at the top of `experiment.py`).

### Launch

```bash
cd Code
python experiment.py
```

A mode selection dialog will appear. Select **Aruco** (vision-based) or **Coordinate** (hard-coded positions), then the main dashboard will open and the experiment will begin automatically.

---

## Captured Frames

Three frames are saved automatically to the working directory during the experiment:

| Filename | Captured at |
|---|---|
| `traffic_light_frame_0.png` | First YELLOW state (filming camera) |
| `traffic_light_frame_1.png` | After Cycle 1, back at filming position |
| `traffic_light_frame_2.png` | After Cycle 2, back at filming position |

---

## Safety Considerations

- Ensure the UR5 workspace is clear of personnel before execution.
- All arm motions should be tested at reduced speed before full experimental runs.
- Sodium hydroxide (NaOH) is corrosive — follow standard laboratory PPE protocols.
- A 30-minute timeout (`TIMEOUT_MINS`) is applied to every colour-transition wait to prevent indefinite blocking if the reaction stalls.

---

## Contributors

Oliver Kenny 
Kinga Dabrowska
Arun Prasath Velkutty

---

## License

[MIT License](LICENSE)
