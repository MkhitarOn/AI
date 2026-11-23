#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// =====================================================
// L298N — ՆՈՐ ՄԻԱՑՈՒՄՆԵՐ
// =====================================================

#define ENA 8
#define IN1 9
#define IN2 10

#define IN3 11
#define IN4 12
#define ENB 13

// =====================================================
// 6 ՀԻՄՆԱԿԱՆ ԳԾԱՅԻՆ ՍԵՆՍՈՐՆԵՐ
// Ձախից աջ՝ 2, 3, 4, 5, 6, 7
// =====================================================

const int sensorPins[6] = {2, 3, 4, 5, 6, 7};
const int sensorWeights[6] = {-6, -4, -1, 1, 4, 6};

// =====================================================
// ԿՈՂԱՅԻՆ ՍԵՆՍՈՐՆԵՐ
// =====================================================

#define LEFT_TURN_SENSOR 17
#define RIGHT_TURN_SENSOR 16

// Սև = HIGH
// Սպիտակ = LOW

// =====================================================
// HC-SR04
// =====================================================

#define TRIG_PIN 22
#define ECHO_PIN 23

// =====================================================
// MPU6050
// =====================================================

int16_t gx, gy, gz;

float yaw = 0.0;
float gz_offset = 0.0;

unsigned long lastTime = 0;

const int GYRO_SIGN = 1;

// =====================================================
// ԳԾԻ ՀԵՏԵՎՄԱՆ PID
// =====================================================

float Kp = 7.0;
float Ki = 2.0;
float Kd = 0.35;

const int MAX_SPEED = 90;
const int BASE_SPEED = 50;
const int SEARCH_SPEED = 70;
const int MIN_SPEED = 40;

// =====================================================
// ՊՏՏՄԱՆ PID
// =====================================================

float KpTurn = 3.7;
float KiTurn = 0.5;
float KdTurn = 1.0;

const int TURN_MIN_PWM = 55;
const int TURN_MAX_PWM = 110;

// =====================================================
// ԱՐԴՅՈՒՆՔՆԵՐ
// =====================================================

enum FollowResult {
  FOLLOWING, LEFT, RIGHT, BOTH_SIDES, WIDE_MARKER, RETURN_MARKER, OBSTACLE, LINE_LOST
};

enum FollowMode { MAIN_FORWARD, SERVER_FORWARD, SERVER_REVERSE };
enum RouteState {
  READY, FOLLOW_MAIN, FOLLOW_SERVER, WAIT_FOR_CHOICE, TURNING,
  PAUSE_AT_OBSTACLE, RETURN_FROM_SERVER, FINISHED, STOPPED, FAULT
};

const unsigned long SERVER_LEG_TIMEOUT_MS = 20000;
const unsigned long LINE_LOSS_TIMEOUT_MS = 1000;
const unsigned long TURN_TIMEOUT_MS = 5000;
const float SERVER_OBSTACLE_CM = 6.0;

RouteState state = READY;
RouteState followAfterChoice = FOLLOW_MAIN;
RouteState followAfterTurn = FOLLOW_MAIN;
unsigned long stateSince = 0;
unsigned long lineLostSince = 0;
bool lineMissing = false;
int turnDirection = 0;
float turnIntegral = 0;
int turnStableCount = 0;
unsigned int mainJunctionCount = 0;
FollowResult pendingMarker = FOLLOWING;

// =====================================================
// ԸՆԴՀԱՆՈՒՐ ՓՈՓՈԽԱԿԱՆՆԵՐ
// =====================================================

float lastLineError = 0.0;

float lineIntegral = 0.0;
float previousLineError = 0.0;

bool junctionArmed = true;

FollowResult candidateMarker = FOLLOWING;
int markerConfirmCount = 0;
int clearConfirmCount = 0;

unsigned long lastSensorTime = 0;
unsigned long lastUltrasonicTime = 0;

float currentDistance = 999.0;


// =====================================================
// ՇԱՐԺԻՉՆԵՐԻ ԿԱՌԱՎԱՐՈՒՄ
// =====================================================

// ՁԱԽ ՇԱՐԺԻՉ
// Ուղղությունները փոխված են

void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Առաջ
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (speed < 0) {
    // Հետ
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, abs(speed));
}

// ԱՋ ՇԱՐԺԻՉ

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Առաջ
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (speed < 0) {
    // Հետ
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(speed));
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// =====================================================
// ԳԻՐՈՍԿՈՊԻ ԿԱԼԻԲՐԱՑՈՒՄ
// =====================================================

void calibrateGyro() {
  long sum = 0;

  Serial.println("Calibrating gyro...");

  for (int i = 0; i < 200; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(5);
  }

  gz_offset = sum / 200.0;

  Serial.print("Gyro offset: ");
  Serial.println(gz_offset);
}

void enterState(RouteState next) {
  stopMotors();
  state = next;
  stateSince = millis();
  lineMissing = false;
  lineIntegral = 0;
  previousLineError = 0;
  lastLineError = 0;
  markerConfirmCount = 0;
  candidateMarker = FOLLOWING;
  clearConfirmCount = 0;
  if (next == RETURN_FROM_SERVER) {
    junctionArmed = false;
  }
}

void beginTurn(int direction, RouteState next) {
  enterState(TURNING);
  turnDirection = direction;
  followAfterTurn = next;
  yaw = 0;
  turnIntegral = 0;
  turnStableCount = 0;
  lastTime = micros();
}

void updateTurn() {
  if (millis() - stateSince >= TURN_TIMEOUT_MS) {
    enterState(FAULT);
    Serial.println(F("TURN TIMEOUT"));
    return;
  }

  unsigned long now = micros();
  unsigned long elapsed = now - lastTime;
  if (elapsed < 4000) {
    return;
  }

  lastTime = now;
  if (elapsed > 50000) {
    turnStableCount = 0;
    stopMotors();
    return;
  }

  mpu.getRotation(&gx, &gy, &gz);
  float dt = elapsed / 1000000.0;
  float rate = GYRO_SIGN * (gz - gz_offset) / 131.0;
  yaw += rate * dt;

  // Left yaw is positive, right yaw is negative.
  float error = 90.0 + turnDirection * yaw;
  float directedRate = -turnDirection * rate;
  if (fabs(error) <= 1.5 && fabs(directedRate) <= 4.0) {
    stopMotors();
    if (++turnStableCount >= 5) {
      Serial.print(F("Final Yaw: "));
      Serial.println(yaw);
      enterState(followAfterTurn);
    }
    return;
  }

  turnStableCount = 0;
  turnIntegral = constrain(turnIntegral + error * dt, -50.0, 50.0);
  float output = KpTurn * error + KiTurn * turnIntegral - KdTurn * directedRate;
  int pwm = 0;
  if (fabs(output) > 0.5) {
    pwm = constrain((int)fabs(output), TURN_MIN_PWM, TURN_MAX_PWM);
  }
  int signedPWM = output >= 0 ? pwm : -pwm;
  motorLeft(turnDirection * signedPWM);
  motorRight(-turnDirection * signedPWM);
}

// =====================================================
// 6 ԳԾԱՅԻՆ ՍԵՆՍՈՐՆԵՐԻ ԸՆԹԵՐՑՈՒՄ
// =====================================================

int readLineSensors(int &blackCount) {

  int pattern = 0;
  blackCount = 0;

  for (int i = 0; i < 6; i++) {

    int value = digitalRead(sensorPins[i]);

    pattern <<= 1;
    pattern |= value;

    if (value == HIGH) {
      blackCount++;
    }
  }

  return pattern;
}

// =====================================================
// HC-SR04 ՀԵՌԱՎՈՐՈՒԹՅՈՒՆ
// =====================================================

float readDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration == 0) {
    return 999.0;
  }

  return duration * 0.0343 / 2.0;
}

// =====================================================
// ՃՅՈՒՂԱՎՈՐՄԱՆ ԵՎ ՓԱԿՈՒՂՈՒ ՀԱՅՏՆԱԲԵՐՈՒՄ
// =====================================================

FollowResult detectJunction(int blackCount, FollowMode mode) {
  FollowResult marker = FOLLOWING;
  if (blackCount >= 5) {
    marker = mode == SERVER_REVERSE ? RETURN_MARKER : WIDE_MARKER;
  }
  else if (mode != SERVER_REVERSE) {
    bool left = digitalRead(LEFT_TURN_SENSOR) == HIGH;
    bool right = digitalRead(RIGHT_TURN_SENSOR) == HIGH;
    if (left && right) {
      marker = BOTH_SIDES;
    }
    else if (left) {
      marker = LEFT;
    }
    else if (right) {
      marker = RIGHT;
    }
  }

  if (!junctionArmed) {
    if (marker == FOLLOWING) {
      if (++clearConfirmCount >= 3) {
        junctionArmed = true;
      }
    }
    else {
      clearConfirmCount = 0;
    }
    return FOLLOWING;
  }

  if (marker != candidateMarker) {
    candidateMarker = marker;
    markerConfirmCount = 0;
  }
  if (marker == FOLLOWING || ++markerConfirmCount < 3) {
    return FOLLOWING;
  }

  junctionArmed = false;
  clearConfirmCount = 0;
  return marker;
}

FollowResult followLine(FollowMode mode) {
  unsigned long now = millis();
  if (now - lastSensorTime < 20) {
    return FOLLOWING;
  }
  lastSensorTime = now;
  bool reverseMode = mode == SERVER_REVERSE;

  if (mode == SERVER_FORWARD && now - lastUltrasonicTime >= 50) {
    lastUltrasonicTime = now;
    currentDistance = readDistance();
    if (currentDistance > 0 && currentDistance <= SERVER_OBSTACLE_CM) {
      stopMotors();
      return OBSTACLE;
    }
  }

  int blackCount = 0;
  int pattern = readLineSensors(blackCount);
  FollowResult marker = detectJunction(blackCount, mode);
  if (marker != FOLLOWING) {
    stopMotors();
    return marker;
  }

  if (blackCount > 0) {
    lineMissing = false;
  }
  else if (!lineMissing) {
    lineMissing = true;
    lineLostSince = now;
  }
  else if (now - lineLostSince >= LINE_LOSS_TIMEOUT_MS) {
    stopMotors();
    return LINE_LOST;
  }

  // Գծի դիրքի հաշվարկ
  float weightedSum = 0;
  int total = 0;

  for (int i = 0; i < 6; i++) {

    bool onLine = pattern & (1 << (5 - i));

    if (onLine) {
      weightedSum += sensorWeights[i];
      total++;
    }
  }

  float error;

  if (total > 0) {

    error = weightedSum / total;
    lastLineError = error;
  }
  else {
    error = lastLineError;
  }

  // PID
  float dt = 0.02;

  float derivative = (error - previousLineError) / dt;

  lineIntegral += error * dt;
  lineIntegral = constrain(lineIntegral, -50.0, 50.0);

  float correction =
    Kp * error +
    Ki * lineIntegral +
    Kd * derivative;

  previousLineError = error;

  correction = constrain(correction, -50, 50);

  int leftSpeed;
  int rightSpeed;

  // Գիծը կորցնելու դեպքում որոնում
  if (total == 0) {

    if (lastLineError >= 0) {

      leftSpeed = SEARCH_SPEED;
      rightSpeed = -SEARCH_SPEED;
    }
    else {

      leftSpeed = -SEARCH_SPEED;
      rightSpeed = SEARCH_SPEED;
    }

    if (reverseMode) {
      leftSpeed = -leftSpeed;
      rightSpeed = -rightSpeed;
    }
  }
  else {

    if (reverseMode) {

      leftSpeed = -BASE_SPEED + correction;
      rightSpeed = -BASE_SPEED - correction;
    }
    else {

      leftSpeed = BASE_SPEED + correction;
      rightSpeed = BASE_SPEED - correction;
    }

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    if (!reverseMode) {

      leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    }
  }

  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  return FOLLOWING;
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // Շարժիչներ
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Գծային սենսորներ
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Կողային սենսորներ
  pinMode(LEFT_TURN_SENSOR, INPUT);
  pinMode(RIGHT_TURN_SENSOR, INPUT);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();

  // MPU6050
  Wire.begin();

  mpu.initialize();

  if (!mpu.testConnection()) {

    Serial.println("MPU6050 connection failed!");

    while (1) {
      stopMotors();
    }
  }

  Serial.println("MPU6050 connected");

  calibrateGyro();

  Serial.println(F("READY: G starts main line; S stops"));
}

// Route decisions live here. Replace this manual policy when the route is agreed.
void applyRouteChoice(char command) {
  if (command == 'F' || command == 'f') {
    enterState(followAfterChoice);
  }
  else if (command == 'L' || command == 'l') {
    beginTurn(-1, followAfterChoice);
  }
  else if (command == 'R' || command == 'r') {
    beginTurn(1, followAfterChoice);
  }
  else if ((command == 'V' || command == 'v') && followAfterChoice == FOLLOW_MAIN &&
           (pendingMarker == LEFT || pendingMarker == BOTH_SIDES || pendingMarker == WIDE_MARKER)) {
    beginTurn(-1, FOLLOW_SERVER);
  }
  else if ((command == 'X' || command == 'x') && followAfterChoice == FOLLOW_MAIN &&
           pendingMarker == WIDE_MARKER) {
    enterState(FINISHED);
    Serial.println(F("FINISHED"));
  }
}

void waitForChoice(FollowResult marker, RouteState resume) {
  pendingMarker = marker;
  followAfterChoice = resume;
  enterState(WAIT_FOR_CHOICE);

  if (resume == FOLLOW_MAIN && marker != WIDE_MARKER) {
    Serial.print(F("Main junction #"));
    Serial.println(++mainJunctionCount);
  }
  switch (marker) {
    case LEFT: Serial.println(F("LEFT")); break;
    case RIGHT: Serial.println(F("RIGHT")); break;
    case BOTH_SIDES: Serial.println(F("BOTH SIDES")); break;
    case WIDE_MARKER: Serial.println(F("WIDE MARKER")); break;
    default: break;
  }

  Serial.println(F("F straight, L left, R right"));
  if (resume == FOLLOW_MAIN && marker != RIGHT) {
    Serial.println(F("V test left server branch"));
  }
  if (resume == FOLLOW_MAIN && marker == WIDE_MARKER) {
    Serial.println(F("X accept finish"));
  }
}

void updateFollowing(FollowMode mode) {
  if (mode != MAIN_FORWARD && millis() - stateSince >= SERVER_LEG_TIMEOUT_MS) {
    enterState(FAULT);
    Serial.println(F("SERVER LEG TIMEOUT"));
    return;
  }

  FollowResult result = followLine(mode);
  switch (result) {
    case FOLLOWING:
      return;
    case LINE_LOST:
      enterState(FAULT);
      Serial.println(F("LINE LOST"));
      return;
    case OBSTACLE:
      enterState(PAUSE_AT_OBSTACLE);
      return;
    case RETURN_MARKER:
      beginTurn(1, FOLLOW_MAIN);
      return;
    default:
      waitForChoice(result, state);
      return;
  }
}

void loop() {
  // Service stop before issuing a new motion command, including during turns.
  char command = 0;
  for (int remaining = Serial.available(); remaining > 0; --remaining) {
    char incoming = Serial.read();
    if (incoming == 'S' || incoming == 's') {
      enterState(STOPPED);
      Serial.println(F("STOPPED: reset to start a new run"));
      return;
    }
    if (incoming != '\r' && incoming != '\n' && command == 0) {
      command = incoming;
    }
  }

  switch (state) {
    case READY:
      if (command == 'G' || command == 'g') {
        enterState(FOLLOW_MAIN);
      }
      break;
    case FOLLOW_MAIN:
      updateFollowing(MAIN_FORWARD);
      break;
    case FOLLOW_SERVER:
      updateFollowing(SERVER_FORWARD);
      break;
    case WAIT_FOR_CHOICE:
      applyRouteChoice(command);
      break;
    case TURNING:
      updateTurn();
      break;
    case PAUSE_AT_OBSTACLE:
      if (millis() - stateSince >= 200) {
        enterState(RETURN_FROM_SERVER);
      }
      break;
    case RETURN_FROM_SERVER:
      updateFollowing(SERVER_REVERSE);
      break;
    case STOPPED:
    case FINISHED:
    case FAULT:
      break;
  }
}
