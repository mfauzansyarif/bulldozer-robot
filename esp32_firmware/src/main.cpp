#include <Arduino.h>
#include <ESP32Servo.h>

#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// WIFI
// ==========================================

const char* ssid = "Fauzan";
const char* password = "abcde12345";

WiFiUDP udp;

const int udpPort = 8888;

char incomingPacket[255];

IPAddress laptopIP;

bool laptopConnected = false;

// ==========================================
// MOTOR PIN
// ==========================================

// LEFT MOTOR
#define PWMA 25
#define AIN1 27
#define AIN2 26

// RIGHT MOTOR
#define PWMB 14
#define BIN1 12
#define BIN2 13

// ==========================================
// SERVO
// ==========================================

#define SERVO_PIN 33

Servo bucketServo;

// ==========================================
// SERVO PARAMETER
// ==========================================

const int SERVO_MIN_ANGLE = 88;
const int SERVO_MAX_ANGLE = 135;

const int SERVO_HOME_ANGLE = 89;

const int SERVO_STEP = 2;

const int SERVO_UPDATE_PERIOD = 5;
// ==========================================
// SERVO VARIABLE
// ==========================================

int currentServoAngle = SERVO_HOME_ANGLE;
int targetServoAngle  = SERVO_HOME_ANGLE;

// ==========================================
// ULTRASONIC
// ==========================================

#define TRIG_PIN 18
#define ECHO_PIN 19

// ==========================================
// ENCODER
// ==========================================

#define ENCODER_LEFT 32
#define ENCODER_RIGHT 34

// ==========================================
// PWM
// ==========================================

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

#define CHANNEL_LEFT 6
#define CHANNEL_RIGHT 7

// ==========================================
// ENCODER VARIABLE
// ==========================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile int currentLeftPWM = 0;
volatile int currentRightPWM = 0;

volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;

// ==========================================
// RPM VARIABLE
// ==========================================

float leftRPM = 0;
float rightRPM = 0;

long prevLeftTicks = 0;
long prevRightTicks = 0;

// ==========================================
// FUNCTION DECLARATION
// ==========================================

void setMotor(int leftPWM, int rightPWM);

float readDistance();

void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  // ==========================================
  // WIFI
  // ==========================================

  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi Connected");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);

  Serial.print("UDP Port: ");
  Serial.println(udpPort);

  // ==========================================
  // MOTOR PIN
  // ==========================================

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // ==========================================
  // PWM SETUP
  // ==========================================

  ledcSetup(CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, CHANNEL_LEFT);

  ledcSetup(CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, CHANNEL_RIGHT);

  // ==========================================
  // SERVO
  // ==========================================

  bucketServo.attach(SERVO_PIN);

  bucketServo.write(
    SERVO_HOME_ANGLE
  );

  currentServoAngle =
    SERVO_HOME_ANGLE;

  targetServoAngle =
    SERVO_HOME_ANGLE;

  // ==========================================
  // ULTRASONIC
  // ==========================================

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ==========================================
  // ENCODER
  // ==========================================

  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_LEFT),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_RIGHT),
    rightEncoderISR,
    RISING
  );

  Serial.println("Bulldozer Ready");
}

// ==========================================
// LOOP
// ==========================================

void loop() {

  // ==========================================
  // UDP COMMAND
  // ==========================================

  int packetSize = udp.parsePacket();

  if (packetSize) {

    int len = udp.read(
      incomingPacket,
      255
    );

    if (len > 0) {

      incomingPacket[len] = 0;

    }

    laptopIP = udp.remoteIP();

    laptopConnected = true;

    int leftPWM = 0;
    int rightPWM = 0;
    int servoAngle = 90;

    int parsed = sscanf(
      incomingPacket,
      "%d,%d,%d",
      &leftPWM,
      &rightPWM,
      &servoAngle
    );

    if (parsed == 3) {

      // ======================================
      // MOTOR
      // ======================================

      setMotor(leftPWM, rightPWM);

      // ======================================
      // SERVO TARGET
      // ======================================

      targetServoAngle = constrain(
        servoAngle,
        SERVO_MIN_ANGLE,
        SERVO_MAX_ANGLE
      );
    }
  }
  // ==========================================
  // SERVO SMOOTHING
  // ==========================================

  static unsigned long lastServoMove = 0;

  if (
    millis() - lastServoMove >
    SERVO_UPDATE_PERIOD
  ) {

    if (
      currentServoAngle <
      targetServoAngle
    ) {

      currentServoAngle +=
        SERVO_STEP;

      if (
        currentServoAngle >
        targetServoAngle
      ) {

        currentServoAngle =
          targetServoAngle;
      }

      bucketServo.write(
        currentServoAngle
      );
    }

    else if (
      currentServoAngle >
      targetServoAngle
    ) {

      currentServoAngle -=
        SERVO_STEP;

      if (
        currentServoAngle <
        targetServoAngle
      ) {

        currentServoAngle =
          targetServoAngle;
      }

      bucketServo.write(
        currentServoAngle
      );
    }

    lastServoMove = millis();
  }

  // ==========================================
  // SERVO DEBUG
  // ==========================================

  static unsigned long lastServoPrint = 0;

  if (
    millis() - lastServoPrint >
    500
  ) {

    Serial.print(
      "Servo Current: "
    );

    Serial.print(
      currentServoAngle
    );

    Serial.print(
      " Target: "
    );

    Serial.println(
      targetServoAngle
    );

    lastServoPrint = millis();
  }
  // ==========================================
  // DISTANCE UDP
  // ==========================================

  static unsigned long lastDistanceSend = 0;

  if (millis() - lastDistanceSend > 100) {

    float distance = readDistance();

    String distMsg;

    if (distance < 0) {

      distMsg = "DIST:999";
    }

    else {

      distMsg = "DIST:" + String(distance);
    }

    if (laptopConnected) {

      udp.beginPacket(
        laptopIP,
        udpPort
      );

      udp.print(distMsg);

      udp.endPacket();
    }

    lastDistanceSend = millis();
  }

  // ==========================================
  // RPM CALCULATION
  // ==========================================

  static unsigned long lastRPMTime = 0;

  if (millis() - lastRPMTime >= 100) {

    long deltaLeft =
      leftTicks - prevLeftTicks;

    long deltaRight =
      rightTicks - prevRightTicks;

    leftRPM =
      (deltaLeft / 40.0) * 600.0;

    rightRPM =
      (deltaRight / 40.0) * 600.0;

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;

    String rpmMsg =
      "RPM:" +
      String(leftRPM) +
      "," +
      String(rightRPM);

    if (laptopConnected) {

      udp.beginPacket(
        laptopIP,
        udpPort
      );

      udp.print(rpmMsg);

      udp.endPacket();
    }

    lastRPMTime = millis();
  }
  // ==========================================
  // ENCODER TELEMETRY
  // ==========================================

  static unsigned long lastEncoderSend = 0;

  if (millis() - lastEncoderSend > 100) {

    String encMsg =
      "ENC:" +
      String(leftTicks) +
      "," +
      String(rightTicks);

    if (laptopConnected) {

      udp.beginPacket(
        laptopIP,
        udpPort
      );

      udp.print(encMsg);

      udp.endPacket();
    }

    lastEncoderSend = millis();
  }
}



// ==========================================
// MOTOR CONTROL
// ==========================================

void setMotor(int leftPWM, int rightPWM) {
  currentLeftPWM = leftPWM;
  currentRightPWM = rightPWM;

  // LEFT MOTOR

  if (leftPWM > 0) {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  else if (leftPWM < 0) {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  else {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  // RIGHT MOTOR

  if (rightPWM > 0) {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  else if (rightPWM < 0) {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  else {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }

  // PWM

  ledcWrite(CHANNEL_LEFT, abs(leftPWM));
  ledcWrite(CHANNEL_RIGHT, abs(rightPWM));
}

// ==========================================
// ULTRASONIC FUNCTION
// ==========================================

float readDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration =
    pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {

    return -1;
  }

  float distance =
    duration * 0.034 / 2;

  if (distance > 400) {

    return -1;
  }

  return distance;
}

// ==========================================
// ENCODER ISR
// ==========================================

void IRAM_ATTR leftEncoderISR()
{
    unsigned long now = micros();

    if (now - lastLeftInterrupt > 300)
    {
        if(currentLeftPWM >= 0)
        {
            leftTicks++;
        }
        else
        {
            leftTicks--;
        }

        lastLeftInterrupt = now;
    }
}

void IRAM_ATTR rightEncoderISR()
{
    unsigned long now = micros();

    if (now - lastRightInterrupt > 300)
    {
        if(currentRightPWM >= 0)
        {
            rightTicks++;
        }
        else
        {
            rightTicks--;
        }

        lastRightInterrupt = now;
    }
}