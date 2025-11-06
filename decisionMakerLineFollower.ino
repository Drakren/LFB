#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int AIN1=2, AIN2=3, PWMA=5;
const int BIN1=4, BIN2=7, PWMB=6;
const int STBY=8;

// ============== PID ==============
int   baseSpeed = 120;     // tune for your bot
float kp = 0.09;
float ki = 0.01;
float kd = 0.10;
float integral = 0;
float error = 0;
float lastError = 0;

int s[SensorCount];
int threshold = 500;       // calibrated scale is 0..1000, so 500 is a good mid

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);  digitalWrite(STBY, HIGH);

  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7,A6,A5,A4,A3,A2,A1,A0}, SensorCount);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Calibrating...");
  for (uint16_t i=0; i<450; i++) {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration Done!");
}

void loop() {
  // readLineBlack() reads calibrated values (0..1000) into sensorValues
  float position = qtr.readLineBlack(sensorValues);   // 0..7000 (8 sensors)
  error = position - 3500.0f;

  // Binary map (1=white, 0=black) on calibrated values
  bool anyBlack = false;
  for (int i=0; i<SensorCount; i++) {
    s[i] = (sensorValues[i] > threshold) ? 1 : 0;   // 1=white, 0=black
    if (s[i] == 0) anyBlack = true;
  }

  // ====== Line lost (all white) -> search spin ======
  if (!anyBlack) {
    // All sensors saw white; spin to find the line
    setMotor(AIN1, AIN2, PWMA,  60);
    setMotor(BIN1, BIN2, PWMB, -60);
    delay(20);
    return; // skip PID this loop
  }

  // ====== Straight segment hint: centers on black ======
  // Since 0 = black, both centers black -> drive straight
  if (s[3] == 0 && s[4] == 0) {
    setMotor(AIN1, AIN2, PWMA, baseSpeed);
    setMotor(BIN1, BIN2, PWMB, baseSpeed);
    // (We still do PID below, or early-return if you want a hard override)
  }

  // ====== PID ======
  // Anti-windup: clamp integral
  integral += error;
  if (integral > 8000) integral = 8000;
  if (integral < -8000) integral = -8000;

  float derivative = error - lastError;
  float correction = kp*error + ki*integral + kd*derivative;
  correction = constrain(correction, -255, 255);

  int leftSpeed  = constrain(baseSpeed + (int)correction, -255, 255);
  int rightSpeed = constrain(baseSpeed - (int)correction, -255, 255);

  setMotor(AIN1, AIN2, PWMA, leftSpeed);
  setMotor(BIN1, BIN2, PWMB, rightSpeed);

  lastError = error;

  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  Err: "); Serial.print(error);
  Serial.print("  L: ");  Serial.print(leftSpeed);
  Serial.print("  R: ");  Serial.println(rightSpeed);

  delay(20);
}

void setMotor(int in1, int in2, int pwm, int speed) {
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  }
}
