#include <Servo.h>

// =====================
// SERVOS
// =====================
Servo servoPan;
Servo servoTilt;
Servo servoShoot;

const int servoPanPin  = 9;
const int servoTiltPin = 10;
const int servoShootPin = 11;
const int ledPin = 8;   // Status / fire indicator LED
const int buzzerPin = 7;

const int loadangle = 55;
// =====================
// STATE
// =====================
int Cur_angle_x = 90;
int Cur_angle_y = 90;

const unsigned long detectionHoldTime = 300;  // milliseconds
unsigned long lastDetectionTime = 0;

void setup() {
  Serial.begin(115200);

  servoPan.attach(servoPanPin);
  servoTilt.attach(servoTiltPin);
  servoShoot.attach(servoShootPin);


  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  servoPan.write(Cur_angle_x);
  servoTilt.write(Cur_angle_y);
  servoShoot.write(loadangle);

  Serial.println("READY");
}

void loop() {

  // =========================
  // SERIAL INPUT CHECK
  // =========================
  if (Serial.available()) {

    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;

    // Update detection time
    lastDetectionTime = millis();

    // Expected: pan,tilt,fire,aggressive
    int i1 = s.indexOf(',');
    int i2 = s.indexOf(',', i1 + 1);
    int i3 = s.indexOf(',', i2 + 1);

    if (i1 < 0 || i2 < 0 || i3 < 0) return;

    int pan = s.substring(0, i1).toInt();
    int tilt = s.substring(i1 + 1, i2).toInt();
    int fire = s.substring(i2 + 1, i3).toInt();
    int aggressive = s.substring(i3 + 1).toInt();

    // Constrain angles
    pan  = constrain(pan,  40, 140);
    tilt = constrain(tilt, 60, 120);

    servoPan.write(pan);
    servoTilt.write(tilt);

    Cur_angle_x = pan;
    Cur_angle_y = tilt;

    digitalWrite(ledPin, aggressive ? HIGH : LOW);

    if (fire) {    
      Serial.println("SHOOT");
      servoShoot.write(0);
    } 
    else {
      servoShoot.write(loadangle);
    }
  }

  // =========================
  // BUZZER CONTROL (SMOOTH)
  // =========================
  if (millis() - lastDetectionTime < detectionHoldTime) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }
}

