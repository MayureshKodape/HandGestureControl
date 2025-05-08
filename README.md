1. Introduction
This project allows you to control 11 servo motors connected to an ESP32 through serial communication. You can send a string of comma-separated angles (ranging from 0 to 180 degrees) via serial, and the ESP32 will adjust the corresponding servos to the specified angles.

This system is useful for robotic applications, remote control devices, and more.

2. Features
Real-Time Servo Control: Adjust up to 11 servo motors using serial commands.

Fast Communication: Uses a baud rate of 250000 for efficient data transmission.

Safety Bounds: Angles are constrained between 0 and 180 degrees to ensure safe servo movement.

Multiple Servo Control: The ESP32 can control multiple servos simultaneously.

3. Tech Stack
Hardware:

ESP32 development board.

11 Servo motors.

Software:

ESP32Servo library for controlling servo motors.

Arduino IDE for programming the ESP32.

4. Hardware Setup
Components Needed:
ESP32 Development Board.

11 Servo motors.

Jumper wires.

External power supply for servos (if required, based on servo current draw).

PC with Arduino IDE installed.

Pin Configuration:
Servo motors are connected to the following GPIO pins of the ESP32:

Servo Number	GPIO Pin
1	4
2	13
3	12
4	14
5	27
6	26
7	33
8	32
9	21
10	19
11	23

Make sure to connect the signal wire of each servo to the respective GPIO pin and provide the VCC and GND connections appropriately.

Power Setup:
Powering the ESP32: Connect the 3.3V and GND of the ESP32 to the power supply for the board.

Powering the Servos: Servos should ideally be powered by an external power supply (e.g., 5V) to prevent overloading the ESP32.

5. Software Setup
Step 1: Install the Arduino IDE
If you haven't already, download and install the Arduino IDE from the official website: Arduino IDE.

Step 2: Install the ESP32 Board in Arduino IDE
Open the Arduino IDE.

Go to File > Preferences and in the "Additional Boards Manager URLs" field, add the following URL:

arduino
Copy
Edit
https://dl.espressif.com/dl/package_esp32_index.json
Go to Tools > Board > Boards Manager and search for ESP32. Click Install.

Step 3: Install the Servo Library
In the Arduino IDE, go to Sketch > Include Library > Manage Libraries.

Search for ESP32Servo and click Install.

Step 4: Upload Code to ESP32
Open the Arduino IDE and load the provided ESP32 servo control code.

Connect your ESP32 to the computer via USB.

In the Tools > Board menu, select your ESP32 board model.

Select the correct Port (under Tools > Port).

Click on the Upload button to upload the code to the ESP32.

6. Usage Instructions
Connect the ESP32 to your PC using a USB cable.

Open a Serial Monitor: Open the Serial Monitor in the Arduino IDE. Set the baud rate to 250000 to match the ESP32 code.

Send Angle Data: In the Serial Monitor, send a string of 11 comma-separated angles (e.g., 90,90,90,90,90,90,90,90,90,90,90), where each number represents the angle for the corresponding servo.

Each angle should be between 0 and 180 degrees.

Servo Movement: The ESP32 will read the angles and move each servo to the specified position.

7. Troubleshooting
Serial Communication Not Working: Ensure that the correct COM port is selected in the Arduino IDE and that the ESP32 is properly connected.

Servos Not Moving: Double-check the servo connections and ensure that the power supply is sufficient for the servos.

Invalid Angle: Ensure that the angles are between 0 and 180. Angles outside this range will be ignored.

