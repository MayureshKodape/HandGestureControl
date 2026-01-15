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
