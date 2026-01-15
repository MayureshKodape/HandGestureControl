# Real-Time Hand-Controlled Robotic Hand using Computer Vision


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Python](https://img.shields.io/badge/Python-3.7+-green.svg)](https://www.python.org/)

A gesture-controlled robotic hand that mimics your hand movements in real-time using 
MediaPipe hand tracking, OpenCV, and ESP32 microcontroller to control 11 servo motors.


##  Demo Video
https://github.com/user-attachments/assets/39974a53-892a-4deb-b1af-48c874ccd37c

https://github.com/user-attachments/assets/326d83a6-f4f7-4d0a-a8a9-3bc010d9fb8e

---

##  Features

- **Real-Time Hand Tracking**: Uses Google's MediaPipe for accurate hand landmark detection
- **11 Servo Control**: Independently controls 11 servos for realistic finger movements
  - Pinky (tip & base)
  - Ring finger (tip & base)
  - Middle finger (tip & base)
  - Index finger (tip & base)
  - Thumb (tip, middle, & spread)
- **High-Speed Communication**: 250,000 baud rate for minimal latency
- **Optimized Performance**: Multi-threaded architecture for smooth real-time control
- **Right Hand Detection**: Automatically detects and tracks right hand movements
- **Safety Features**: Angle constraints (0-180°) to prevent servo damage
- **Low Latency**: Optimized camera settings and processing for responsive control

---

##  Tech Stack

### Hardware
- **ESP32 Development Board** - Microcontroller for servo control
- **11 x Servo Motors** (e.g., SG90 or MG996R) - Actuators for finger movement
- **Webcam** - For hand tracking
- **Power Supply** - 5V external power for servos (recommended)
- **USB Cable** - For ESP32 programming and serial communication
- **Jumper Wires** - For connections

### Software
- **Python 3.7+** - Main programming language
- **OpenCV** - Computer vision and camera interface
- **MediaPipe** - Hand tracking and landmark detection
- **PySerial** - Serial communication with ESP32
- **Arduino IDE** - ESP32 programming
- **ESP32Servo Library** - Servo motor control

---

##  System Architecture
```
┌─────────────────┐      USB Serial       ┌──────────────┐
│   Python Script │ ──────250000 baud────► │    ESP32     │
│  (Hand Tracker) │      (Angle Data)      │ Microcontroller│
└─────────────────┘                        └──────────────┘
        │                                          │
        │                                          │
    Webcam                                    11 Servos
   (Camera)                                  (Robotic Hand)
        │                                          │
        ▼                                          ▼
  ┌──────────┐                            ┌───────────────┐
  │Your Hand │                            │Robotic Hand   │
  │Movements │                            │Mimics Actions │
  └──────────┘                            └───────────────┘
```

### How It Works

1. **Hand Detection**: Webcam captures your hand movements
2. **Landmark Extraction**: MediaPipe identifies 21 hand landmarks
3. **Angle Calculation**: Python calculates finger joint angles from landmarks
4. **Mapping**: Angles are mapped from hand coordinates (30-160°) to servo range (0-180°)
5. **Serial Transmission**: Angle data sent to ESP32 as comma-separated values
6. **Servo Control**: ESP32 updates all 11 servos simultaneously
7. **Real-Time Feedback**: Process repeats at high FPS for smooth movement

---

## Hardware Setup

### Pin Configuration

Connect servo motors to the ESP32 GPIO pins as follows:

| Servo Number | Finger Joint      | ESP32 GPIO Pin |
|--------------|-------------------|----------------|
| 1            | Pinky Tip         | 4              |
| 2            | Pinky Base        | 13             |
| 3            | Ring Tip          | 12             |
| 4            | Ring Base         | 14             |
| 5            | Middle Tip        | 27             |
| 6            | Middle Base       | 26             |
| 7            | Index Tip         | 33             |
| 8            | Index Base        | 32             |
| 9            | Thumb Tip         | 21             |
| 10           | Thumb Middle      | 19             |
| 11           | Thumb Spread      | 23             |

### Wiring Diagram
```
ESP32                    Servo Motors
─────                    ────────────
GPIO 4  ──────────────►  Pinky Tip (Signal)
GPIO 13 ──────────────►  Pinky Base (Signal)
GPIO 12 ──────────────►  Ring Tip (Signal)
GPIO 14 ──────────────►  Ring Base (Signal)
GPIO 27 ──────────────►  Middle Tip (Signal)
GPIO 26 ──────────────►  Middle Base (Signal)
GPIO 33 ──────────────►  Index Tip (Signal)
GPIO 32 ──────────────►  Index Base (Signal)
GPIO 21 ──────────────►  Thumb Tip (Signal)
GPIO 19 ──────────────►  Thumb Middle (Signal)
GPIO 23 ──────────────►  Thumb Spread (Signal)

5V Power Supply ──────►  All Servos VCC (Red Wire)
GND ───────────────────►  All Servos GND (Brown/Black Wire)
```

### Important Power Considerations

- **DO NOT** power all servos directly from ESP32's 5V pin
- Use an **external 5V power supply** (2-3A recommended)
- Connect ESP32 GND to power supply GND (common ground)
- Each servo draws 100-500mA under load; 11 servos can draw 1-5A total

---

##  Software Setup

### Step 1: Install Arduino IDE

1. Download from [Arduino Official Website](https://www.arduino.cc/en/software)
2. Install following the platform-specific instructions

### Step 2: Configure ESP32 in Arduino IDE

1. Open Arduino IDE
2. Go to `File` → `Preferences`
3. Add this URL to "Additional Boards Manager URLs":
```
   https://dl.espressif.com/dl/package_esp32_index.json
```
4. Go to `Tools` → `Board` → `Boards Manager`
5. Search for "ESP32" and click **Install**

### Step 3: Install ESP32Servo Library

1. Go to `Sketch` → `Include Library` → `Manage Libraries`
2. Search for "ESP32Servo"
3. Click **Install**

### Step 4: Upload ESP32 Code

1. Connect ESP32 to your computer via USB
2. Open the provided ESP32 code in Arduino IDE
3. Select your board: `Tools` → `Board` → `ESP32 Dev Module` (or your specific model)
4. Select the correct port: `Tools` → `Port` → `COM X` (Windows) or `/dev/cu.usbserial-XXX` (Mac)
5. Click **Upload** button (→)
6. Wait for "Done uploading" message

### Step 5: Install Python Dependencies
```bash
# Install required packages
pip install opencv-python mediapipe pyserial numpy

# Or use requirements.txt
pip install -r requirements.txt
```

Create a `requirements.txt` file:
```
opencv-python>=4.5.0
mediapipe>=0.10.0
pyserial>=3.5
numpy>=1.21.0
```

### Step 6: Configure Serial Port

Edit the Python script to match your serial port:

**Windows:**
```python
ser = serial.Serial('COM3', 250000, timeout=0)  # Change COM3 to your port
```

**Mac/Linux:**
```python
ser = serial.Serial('/dev/tty.usbserial-110', 250000, timeout=0)  # Update port name
```

To find your port:
- **Windows**: Device Manager → Ports (COM & LPT)
- **Mac**: `ls /dev/tty.*` in Terminal
- **Linux**: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`

---

##  Usage Instructions

### Running the System

1. **Connect Hardware**
   - Connect ESP32 via USB
   - Ensure servos are properly wired and powered

2. **Upload ESP32 Code**
```bash
   # Already done in software setup
```

3. **Run Python Hand Tracker**
```bash
   python hand_tracking.py
```

4. **Position Your Hand**
   - Place your **right hand** in front of the webcam
   - Keep hand within camera frame
   - Ensure good lighting for best tracking

5. **Watch the Magic! ✨**
   - Move your fingers - the robotic hand will follow
   - Check console for FPS readings
   - Press **'q'** to quit

### Expected Behavior

- **FPS Display**: Console shows real-time FPS (aim for 15-30 FPS)
- **Immediate Response**: Robotic hand mirrors your movements with minimal delay
- **Smooth Motion**: Servos update only when angles change to reduce jitter

---

##  Gesture Examples

Try these gestures to test your robotic hand:

| Gesture | Description |
|---------|-------------|
| ✊ Fist | Close all fingers - robotic hand makes a fist |
| ✋ Open Hand | Extend all fingers - robotic hand opens fully |
| ✌️ Peace Sign | Index and middle finger up - robot mimics |
| 👍 Thumbs Up | Thumb extended, others closed |
| 🤏 Pinch | Thumb and index together - precise control |
| 🤘 Rock On | Pinky and index up, others down |

---

##  Troubleshooting

### Common Issues

#### Serial Port Errors
```
Serial error: could not open port...
```
**Solutions:**
- Check ESP32 is connected via USB
- Update serial port name in Python script
- Ensure no other program is using the port (close Arduino Serial Monitor)
- Try unplugging and reconnecting ESP32

#### Hand Not Detected
```
No hand landmarks detected
```
**Solutions:**
- Ensure good lighting
- Show your **right hand** (code configured for right hand)
- Keep hand within camera frame
- Try adjusting `min_detection_confidence` (default: 0.5)

#### Low FPS
```
FPS: 8.5
```
**Solutions:**
- Close other applications
- Lower camera resolution (already optimized to 240x180)
- Reduce `max_num_hands` to 1 (already set)
- Check CPU usage

#### Servos Not Moving
**Solutions:**
- Verify servo connections to correct GPIO pins
- Check power supply is connected and adequate (5V, 2A+)
- Test individual servos with Arduino examples
- Ensure serial baud rate matches (250000)

#### Jittery Movement
**Solutions:**
- Improve lighting conditions
- Keep hand steady
- Add smoothing filter (optional - uncomment filtering code)
- Increase `min_tracking_confidence`

#### Import Errors
```
ModuleNotFoundError: No module named 'cv2'
```
**Solutions:**
```bash
pip install opencv-python
pip install mediapipe
pip install pyserial
```

---

##  Performance Optimization

### Current Optimizations

- **Low Resolution**: 240x180 camera capture
- **High Frame Rate**: 60 FPS camera setting
- **Multi-Threading**: Separate threads for camera and processing
- **Minimal Buffer**: Buffer size = 1 to reduce lag
- **Conditional Updates**: Servos update only when angles change
- **Single Hand**: Tracks max 1 hand for speed

### Further Optimization Options
```python
# Increase confidence for faster but less sensitive tracking
min_detection_confidence=0.7
min_tracking_confidence=0.7

# Add smoothing filter
from scipy.signal import savgol_filter
angles_smooth = savgol_filter(angles, window_length=5, polyorder=2)
```

---

##  Future Improvements

- [ ] **Left Hand Support**: Add left hand tracking option
- [ ] **Gesture Recognition**: Detect specific gestures (peace sign, thumbs up, etc.)
- [ ] **Wireless Control**: Replace serial with WiFi/Bluetooth
- [ ] **Both Hands**: Control two robotic hands simultaneously
- [ ] **Force Feedback**: Add sensors to detect grip strength
- [ ] **GUI Interface**: Control panel with calibration options
- [ ] **Recording Mode**: Record and replay gesture sequences
- [ ] **Machine Learning**: Train custom gestures

---

## 📁 Project Structure
```
robotic-hand-control/
│
├── hand_tracking.py          # Main Python hand tracking script
├── esp32_servo_control.ino   # ESP32 Arduino code
├── requirements.txt          # Python dependencies
├── README.md                 # This file
│
├── demo/                     # Demo videos and images
│   ├── demo_video.mp4
│   └── setup_photo.jpg
│
├── docs/                     # Additional documentation
│   ├── wiring_diagram.png
│   └── calibration_guide.md
│
└── 3d_models/               # 3D printable files (optional)
    └── robotic_hand.stl
```

---

##  Contributing

Contributions are welcome! Feel free to:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---
