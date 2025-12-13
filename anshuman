// ---------- PIN DEFINITIONS ----------

// 16-Sensor IR Array (Multiplexed)
#define LEd_ir  12
#define S0      19
#define S1      18
#define S2      15
#define S3      16
#define LED     13
int ANALOG_IN = A3;

// IR Sensor Values
int sensorAnalog[16];
int sensorMin[16];
int sensorMax[16];

// ---------- MOTOR PINS (TB6612) ----------
// Left Motor
#define L_IN1 4
#define L_IN2 5
#define L_PWM 6

// Right Motor
#define R_IN1 7
#define R_IN2 8
#define R_PWM 9

// ---------- PID PARAMETERS ----------
float Kp = 0.25;
float Kd = 2.2;
float lastError = 0;

// ---------- SPEED ----------
int baseSpeed = 150;   



// ======================================================
//                      SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(LEd_ir, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LEd_ir, HIGH);

  // Motor pins
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // Initialize min/max arrays
  for(int i=0;i<16;i++){
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.println("Calibrating sensors...");

  // ---- 500-loop Calibration ----
  for(int t = 0; t < 500; t++){
    readSensors();

    for(int i = 0; i < 16; i++){
      if(sensorAnalog[i] < sensorMin[i]) sensorMin[i] = sensorAnalog[i];
      if(sensorAnalog[i] > sensorMax[i]) sensorMax[i] = sensorAnalog[i];
    }
  }

  Serial.println("Calibration Done!");
}



// ======================================================
//                 SENSOR READ FUNCTION
// ======================================================
void readSensors() {
  for (int i = 0; i < 16; i++) {

    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);
    delayMicroseconds(5);

    sensorAnalog[i] = analogRead(ANALOG_IN);

    // Normalize 0–1000
    sensorAnalog[i] = map(sensorAnalog[i],
                          sensorMin[i], sensorMax[i],
                          0, 1000);
    sensorAnalog[i] = constrain(sensorAnalog[i], 0, 1000);
  }
}



// ======================================================
//           POSITION CALCULATION (0–15000)
// ======================================================
long getLinePosition() {
  long weightedSum = 0;
  long total = 0;

  for(int i = 0; i < 16; i++){
    int value = 1000 - sensorAnalog[i];  // black = higher weight
    weightedSum += (long)value * (i * 1000);   // sensor index * 1000
    total += value;
  }

  if (total == 0) return -1;   // line lost

  return weightedSum / total;  // position (0–15000)
}



// ======================================================
//                       MOTOR CONTROL
// ======================================================
void setMotorLeft(int speed){
  if(speed >= 0){
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, speed);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, -speed);
  }
}

void setMotorRight(int speed){
  if(speed >= 0){
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_PWM, speed);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_PWM, -speed);
  }
}



// ======================================================
//                         LOOP
// ======================================================
void loop() {

  readSensors();

  long pos = getLinePosition();

  if(pos == -1){
    // Line lost → stop or search
    setMotorLeft(0);
    setMotorRight(0);
    return;
  }

  int error = pos - 7500;   // 0–15000 → midpoint 7500

  // ---- PD CONTROL ----
  float P = Kp * error;
  float D = Kd * (error - lastError);
  float correction = P + D;
  lastError = error;

  // ---- Apply to motors ----
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
}
