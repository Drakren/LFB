// ---- QTR Sensor Pins ----
int qtrPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ---- Motor Pins ----
// Left Motor (Driver 1)
#define L_IN1 4
#define L_IN2 5
#define L_PWM 6

// Right Motor (Driver 2)
#define R_IN1 7
#define R_IN2 8
#define R_PWM 9

// ---- Motor Speed ----
int baseSpeed = 140;   // change depending on motor power

void setup() {
  Serial.begin(9600);

  // Motor pin setup
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // QTR pins as input
  for(int i = 0; i < 8; i++){
    pinMode(qtrPins[i], INPUT);
  }
}

void loop() {

  int s[8];

  // Read all QTR sensors
  for(int i = 0; i < 8; i++){
    s[i] = analogRead(qtrPins[i]);
  }

  // Print values (for debugging)
  for(int i = 0; i < 8; i++){
    Serial.print(s[i]); Serial.print(" ");
  }
  Serial.println();

  // ---- Basic Line Following Logic ----
  // Sensors will give LOW value on black line and HIGH on white (depends on wiring)
  // Tune threshold accordingly.
  int threshold = 500;

  bool leftSide = (s[0] < threshold || s[1] < threshold || s[2] < threshold);
  bool rightSide = (s[5] < threshold || s[6] < threshold || s[7] < threshold);
  bool center = (s[3] < threshold || s[4] < threshold);

  if(center){
    forward(baseSpeed);
  }
  else if(leftSide){
    turnLeft(baseSpeed);
  }
  else if(rightSide){
    turnRight(baseSpeed);
  }
  else {
    // No line detected → stop or search
    stopMotors();
  }
}

// ---- Motor Functions ----

void forward(int speed){
  // Left motor
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, speed);

  // Right motor
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, speed);
}

void turnLeft(int speed){
  // Left motor slow/stop
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 0);

  // Right motor forward
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, speed);
}

void turnRight(int speed){
  // Left motor forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, speed);

  // Right motor slow/stop
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 0);
}

void stopMotors(){
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 0);

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 0);
}
