#include <Servo.h>

// ESCs connected to analog pins A0ï¿½A3
Servo frontRight;
Servo rearRight;
Servo rearLeft;
Servo frontLeft;

int valFR = 1500, valRR = 1500, valRL = 1500, valFL = 1500;

void setup() {
  Serial.begin(115200);

  frontRight.attach(14);  // A0
  rearRight.attach(15);   // A1
  rearLeft.attach(16);    // A2
  frontLeft.attach(17);   // A3

  // Set all ESCs to neutral at start
  frontRight.writeMicroseconds(1500);
  rearRight.writeMicroseconds(1500);
  rearLeft.writeMicroseconds(1500);
  frontLeft.writeMicroseconds(1500);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseCommand(line);
    updateMotors();
  }
}

void parseCommand(String cmd) {
  int idx;

  idx = cmd.indexOf("FR:");
  if (idx >= 0) valFR = mapValue(cmd.substring(idx + 3));

  idx = cmd.indexOf("RR:");
  if (idx >= 0) valRR = mapValue(cmd.substring(idx + 3));

  idx = cmd.indexOf("RL:");
  if (idx >= 0) valRL = mapValue(cmd.substring(idx + 3));

  idx = cmd.indexOf("FL:");
  if (idx >= 0) valFL = mapValue(cmd.substring(idx + 3));
}

int mapValue(String val) {
  int v = val.toInt();
  v = constrain(v, -255, 255);

  if (abs(v) < 5) return 1500;  // Stop motor if value is near 0

  return map(v, -255, 255, 1000, 2000);  // Map to microsecond PWM
}

void updateMotors() {
  frontRight.writeMicroseconds(valFR);
  rearRight.writeMicroseconds(valRR);
  rearLeft.writeMicroseconds(valRL);
  frontLeft.writeMicroseconds(valFL);
}
