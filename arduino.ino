// #include <ESP32Servo.h>

// Servo servos[11];
// int servoPins[11] = {4, 13, 12, 14, 27, 26, 33, 32, 21, 19,23}; // Your servo pin numbers
// int currentAngles[11] = {50};  // Initial neutral angles





// void setup() {
//  Serial.begin(115200);

//  for (int i = 0; i < 11; i++) {
//    servos[i].attach(servoPins[i]);
//    servos[i].write(currentAngles[i]);
//  }
// }

// void loop() {
//  if (Serial.available()) {
//    String data = Serial.readStringUntil('\n');
//    data.trim();

//    int angles[11] = {0};
//    int angleIndex = 0;
//    String temp = "";

//    for (int i = 0; i < data.length(); i++) {
//      if (data[i] == ',') {
//        if (angleIndex < 11 && temp.length() > 0) {
//          angles[angleIndex] = constrain(temp.toInt(), 0, 180);
//          temp = "";
//          angleIndex++;
//        }
//      } else {
//        temp += data[i];
//      }
//    }

//    // Add last value
//    if (angleIndex < 11 && temp.length() > 0) {
//      angles[angleIndex] = constrain(temp.toInt(), 0, 180);
//    }

//    // Apply angles with basic smoothing (optional)
//    for (int i = 0; i < 11; i++) {
//      currentAngles[i] = smoothAngle(currentAngles[i], angles[i]);
//      servos[i].write(currentAngles[i]);
//    }
//  }
// }

// int smoothAngle(int current, int target) {
//  int step = 3;  // Smaller = smoother but slower response
//  if (abs(current - target) <= step) return target;
//  return current + (target > current ? step : -step);
// }
/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-servo-motor-web-server-arduino-ide/
  Based on the ESP32Servo Sweep Example
*********/

// #include <ESP32Servo.h>

// static const int servoPin = 33;

// Servo servo1;

// void setup() {

//   Serial.begin(115200);
//   servo1.attach(servoPin);
// }

// void loop() {
//   for(int posDegrees = 60; posDegrees <= 180; posDegrees++) {
//     servo1.write(posDegrees);
//     Serial.println(posDegrees);
//     delay(20);
//   }

//   for(int posDegrees = 180; posDegrees >= 60; posDegrees--) {
//     servo1.write(posDegrees);
//     Serial.println(posDegrees);
//     delay(20);
//   }
// }

#include <ESP32Servo.h>

Servo servos[11];
int servoPins[11] = {4, 13, 12, 14, 27, 26, 33, 32, 21, 19, 23};

void setup() {
  Serial.begin(250000);  // Use 250000 baud for faster serial
  for (int i = 0; i < 11; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);  // Set to neutral angle initially
  }
}

void loop() {
  static char buffer[64];  // Enough for 11 angles and commas
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';  // Null-terminate the string

    char *token = strtok(buffer, ",");
    int i = 0;
    while (token != NULL && i < 11) {
      int angle = atoi(token);
      angle = constrain(angle, 0, 180);  // Safety bounds
      servos[i].write(angle);  // Direct update without smoothing
      token = strtok(NULL, ",");
      i++;
    }
  }
}
