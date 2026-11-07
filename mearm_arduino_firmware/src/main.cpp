#include <Arduino.h>
#include <Servo.h>

#include "protocol.h"

// Pin map
static const int BASE_SERVO_PIN = 6;
static const int SHOULDER_SERVO_PIN = 9;
static const int ELBOW_SERVO_PIN = 10;
static const int CLAW_SERVO_PIN = 11;

static const int SETUP_PIN = 2;
static const int LED_PIN = LED_BUILTIN;

// Servo objects
Servo g_base_servo;
Servo g_shoulder_servo;
Servo g_elbow_servo;
Servo g_claw_servo;

// PWM limits (microseconds). Using the Arduino Servo library default range
// gives the widest compatibility with the TowerPro 9g / MG90s servos.
static const int SERVO_MIN_US = 544;
static const int SERVO_MAX_US = 2400;

// Continuous-rotation base limits (microseconds)
static const int BASE_STOP_US = 1500;
static const int BASE_MIN_US = 1000;
static const int BASE_MAX_US = 2000;

// Physical servo angle limits for position joints (radians).
// These match the URDF-mapped ranges used by the ROS driver.
static const float SHOULDER_MIN_RAD = 0.5707963f;  // 32.7 deg
static const float SHOULDER_MAX_RAD = M_PI;        // 180 deg
static const float ELBOW_MIN_RAD = 0.0f;           // 0 deg
static const float ELBOW_MAX_RAD = M_PI_2;         // 90 deg
static const float CLAW_CLOSED_RAD = 1.39626f;     // closed, 80 deg
static const float CLAW_OPEN_RAD = 1.65806f;       // open, 95 deg

// Watchdog
static const uint32_t WATCHDOG_TIMEOUT_MS = 500;
static uint32_t g_last_command_ms = 0;

// Last commanded values (position joints are physical servo angles in radians)
static float g_cmd_base_speed = 0.0f;
static float g_cmd_shoulder_rad = M_PI_2;    // 90 deg physical
static float g_cmd_elbow_rad = M_PI_2;       // 90 deg physical
static float g_cmd_claw_rad = CLAW_OPEN_RAD; // open, 95 deg physical

static mearm_protocol::FrameParser g_parser;

static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static int physicalAngleToMicroseconds(float angle_rad) {
  // Convert a physical servo angle (0..180 deg) directly to pulse width.
  const float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  const float t = clampf(angle_deg / 180.0f, 0.0f, 1.0f);
  return static_cast<int>(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
}

static int speedToMicroseconds(float speed) {
  speed = clampf(speed, -1.0f, 1.0f);
  return static_cast<int>(BASE_STOP_US + speed * (BASE_MAX_US - BASE_STOP_US));
}

static void writeServos() {
  g_base_servo.writeMicroseconds(speedToMicroseconds(g_cmd_base_speed));
  g_shoulder_servo.writeMicroseconds(
      physicalAngleToMicroseconds(g_cmd_shoulder_rad));
  g_elbow_servo.writeMicroseconds(physicalAngleToMicroseconds(g_cmd_elbow_rad));
  g_claw_servo.writeMicroseconds(physicalAngleToMicroseconds(g_cmd_claw_rad));
}

static void enterSetupMode() {
  Serial.println(
      "SETUP MODE: attach arms at shoulder/elbow 90 deg, claw closed.");
  Serial.println("Reset the Arduino when done.");

  // Base stopped, shoulder/elbow at 90 deg physical, claw closed
  g_base_servo.writeMicroseconds(BASE_STOP_US);
  g_shoulder_servo.writeMicroseconds(physicalAngleToMicroseconds(M_PI_2));
  g_elbow_servo.writeMicroseconds(physicalAngleToMicroseconds(M_PI_2));
  g_claw_servo.writeMicroseconds(physicalAngleToMicroseconds(CLAW_CLOSED_RAD));

  for (;;) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}

static void handleServoCommand(const mearm_protocol::ServoCmd &cmd) {
  if (isnan(cmd.joint_0) || isnan(cmd.joint_1) || isnan(cmd.joint_2) ||
      isnan(cmd.joint_3)) {
    return;
  }

  g_cmd_base_speed = cmd.joint_0;
  g_cmd_shoulder_rad = clampf(cmd.joint_1, SHOULDER_MIN_RAD, SHOULDER_MAX_RAD);
  g_cmd_elbow_rad = clampf(cmd.joint_2, ELBOW_MIN_RAD, ELBOW_MAX_RAD);
  g_cmd_claw_rad = clampf(cmd.joint_3, CLAW_CLOSED_RAD, CLAW_OPEN_RAD);
  g_last_command_ms = millis();

  writeServos();
}

static void pollSerial() {
  while (Serial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    const auto result = g_parser.feed(byte);

    if (result == mearm_protocol::FrameParser::Result::FrameReady) {
      if (g_parser.type() == mearm_protocol::PacketType::ServoCmdPacket &&
          g_parser.length() == sizeof(mearm_protocol::ServoCmd)) {
        mearm_protocol::ServoCmd cmd;
        memcpy(&cmd, g_parser.payload(), sizeof(cmd));
        handleServoCommand(cmd);
      }
      g_parser.reset();
    } else if (result == mearm_protocol::FrameParser::Result::InvalidFrame) {
      g_parser.reset();
    }
  }
}

static void checkWatchdog() {
  if ((millis() - g_last_command_ms) > WATCHDOG_TIMEOUT_MS) {
    // Stop the base, hold last commanded arm position
    g_cmd_base_speed = 0.0f;
    g_base_servo.writeMicroseconds(BASE_STOP_US);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(SETUP_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  g_base_servo.attach(BASE_SERVO_PIN);
  g_shoulder_servo.attach(SHOULDER_SERVO_PIN);
  g_elbow_servo.attach(ELBOW_SERVO_PIN);
  g_claw_servo.attach(CLAW_SERVO_PIN);

  // Start from a safe position
  writeServos();

  if (digitalRead(SETUP_PIN) == LOW) {
    enterSetupMode();
  }

  g_last_command_ms = millis();
  Serial.println("MeArm Arduino firmware ready.");
}

void loop() {
  pollSerial();
  checkWatchdog();
}
