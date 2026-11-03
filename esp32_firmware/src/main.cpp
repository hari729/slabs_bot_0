#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "protocol.h"

// Pin map
static const int LEFT_PWM_PIN = 19;
static const int LEFT_DIR_PIN = 18;
static const int RIGHT_PWM_PIN = 26;
static const int RIGHT_DIR_PIN = 25;

static const int RIGHT_ENC_A_PIN = 32;
static const int RIGHT_ENC_B_PIN = 33;
static const int LEFT_ENC_A_PIN = 34;
static const int LEFT_ENC_B_PIN = 35;

static const int ESTOP_PIN = 4;

// MPU6050 I2C
static const int MPU6050_SDA_PIN = 21;
static const int MPU6050_SCL_PIN = 22;
static const uint8_t MPU6050_ADDR = 0x68;

static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_REG_CONFIG = 0x1A;
static const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

static const uint8_t MPU6050_PWR_MGMT_1_RESET = 0x80;
static const uint8_t MPU6050_PWR_MGMT_1_WAKE_CLK_PLL = 0x01;
static const uint8_t MPU6050_ACCEL_CONFIG_FS_2G = 0x00;
static const uint8_t MPU6050_GYRO_CONFIG_FS_250DPS = 0x00;
static const uint8_t MPU6050_CONFIG_DLPF_44HZ = 0x03;

static const uint32_t MPU6050_I2C_CLOCK_HZ = 400000;

static const int LEFT_ENCODER_SIGN = -1;
static const int RIGHT_ENCODER_SIGN = 1;

static const float TICKS_PER_REVOLUTION = 1399.2f;

static const uint32_t CONTROL_PERIOD_MS = 10;
static const uint32_t REPORT_PERIOD_MS = 20;
static const uint32_t WATCHDOG_TIMEOUT_MS = 500;

static const uint32_t CONTROL_TASK_STACK_WORDS = 4096;
static const uint32_t REPORT_TASK_STACK_WORDS = 3072;
static const uint32_t SERIAL_TASK_STACK_WORDS = 4096;

static const UBaseType_t CONTROL_TASK_PRIORITY = 3;
static const UBaseType_t REPORT_TASK_PRIORITY = 2;
static const UBaseType_t SERIAL_TASK_PRIORITY = 2;

static const TickType_t MUTEX_TIMEOUT_TICKS = pdMS_TO_TICKS(50);

static const int PWM_FREQUENCY_HZ = 20000;
static const int PWM_RESOLUTION_BITS = 10;
static const int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
static const int LEFT_PWM_CHANNEL = 0;
static const int RIGHT_PWM_CHANNEL = 1;

static const float FF_DUTY_PER_RPM = 7.522;
static const float MIN_START_DUTY = 98.0;
static const float PI_KP = 2.051;
static const float PI_KI = 20.515;
static const float PI_INTEGRAL_LIMIT = 100.0f;
static const float MAX_ABS_RPM = 150.0f;

ESP32Encoder g_left_encoder;
ESP32Encoder g_right_encoder;

float g_target_left_rpm = 0.0f;
float g_target_right_rpm = 0.0f;
float g_measured_left_rpm = 0.0f;
float g_measured_right_rpm = 0.0f;
float g_integral_left = 0.0f;
float g_integral_right = 0.0f;

int32_t g_last_left_ticks = 0;
int32_t g_last_right_ticks = 0;

uint32_t g_last_command_ms = 0;
bool g_watchdog_tripped = false;

static SemaphoreHandle_t g_state_mutex = nullptr;
static SemaphoreHandle_t g_serial_mutex = nullptr;

static TaskHandle_t g_control_task_handle = nullptr;
static TaskHandle_t g_report_task_handle = nullptr;
static TaskHandle_t g_serial_task_handle = nullptr;

static bool g_mpu6050_initialized = false;

static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static int32_t read_encoder_ticks(ESP32Encoder &encoder, int sign) {
  return static_cast<int32_t>(encoder.getCount()) * sign;
}

static void set_motor_pwm(int pwm_channel, int dir_pin, int duty) {
  duty = constrain(duty, -PWM_MAX_DUTY, PWM_MAX_DUTY);

  if (duty >= 0) {
    digitalWrite(dir_pin, LOW);
    ledcWrite(pwm_channel, duty);
  } else {
    digitalWrite(dir_pin, HIGH);
    ledcWrite(pwm_channel, PWM_MAX_DUTY + duty);
  }
}

static void stop_motors_locked(bool take_mutex) {
  if (take_mutex) {
    if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) != pdTRUE) {
      set_motor_pwm(LEFT_PWM_CHANNEL, LEFT_DIR_PIN, 0);
      set_motor_pwm(RIGHT_PWM_CHANNEL, RIGHT_DIR_PIN, 0);
      return;
    }
  }

  g_target_left_rpm = 0.0f;
  g_target_right_rpm = 0.0f;
  g_integral_left = 0.0f;
  g_integral_right = 0.0f;
  set_motor_pwm(LEFT_PWM_CHANNEL, LEFT_DIR_PIN, 0);
  set_motor_pwm(RIGHT_PWM_CHANNEL, RIGHT_DIR_PIN, 0);

  if (take_mutex) {
    xSemaphoreGive(g_state_mutex);
  }
}

static bool write_mpu6050_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool read_mpu6050_registers(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t received = Wire.requestFrom(static_cast<int>(MPU6050_ADDR), static_cast<int>(len));
  if (received < len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

static int16_t build_int16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low));
}

static bool init_mpu6050() {
  Wire.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN, MPU6050_I2C_CLOCK_HZ);

  // Reset the device and wait for reset to clear.
  if (!write_mpu6050_register(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET)) {
    return false;
  }
  delay(100);

  // Wake up and use PLL with X-axis gyro reference.
  if (!write_mpu6050_register(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE_CLK_PLL)) {
    return false;
  }

  // Configure DLPF for ~44 Hz gyro bandwidth.
  if (!write_mpu6050_register(MPU6050_REG_CONFIG, MPU6050_CONFIG_DLPF_44HZ)) {
    return false;
  }

  // Accel full scale: +/- 2 g.
  if (!write_mpu6050_register(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_FS_2G)) {
    return false;
  }

  // Gyro full scale: +/- 250 deg/s.
  if (!write_mpu6050_register(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_250DPS)) {
    return false;
  }

  // Verify device identity by reading WHO_AM_I (0x75).
  uint8_t who_am_i = 0;
  if (!read_mpu6050_registers(0x75, &who_am_i, 1)) {
    return false;
  }
  if (who_am_i != 0x68) {
    return false;
  }

  return true;
}

static bool read_mpu6050_raw(int16_t accel[3], int16_t gyro[3]) {
  uint8_t buffer[14];
  if (!read_mpu6050_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }

  accel[0] = build_int16(buffer[0], buffer[1]);
  accel[1] = build_int16(buffer[2], buffer[3]);
  accel[2] = build_int16(buffer[4], buffer[5]);

  // Skip temperature bytes at buffer[6..7].
  gyro[0] = build_int16(buffer[8], buffer[9]);
  gyro[1] = build_int16(buffer[10], buffer[11]);
  gyro[2] = build_int16(buffer[12], buffer[13]);

  return true;
}

static bool estop_is_pressed() {
  if (ESTOP_PIN < 0) return false;
  return digitalRead(ESTOP_PIN) == LOW;
}

static int pi_to_pwm(float target_rpm, float measured_rpm, float &integral,
                     float dt_s) {
  if (fabsf(target_rpm) < 1.0f) {
    integral = 0.0f;
    return 0;
  }

  const float error = target_rpm - measured_rpm;
  integral += error * dt_s;
  integral = clampf(integral, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);

  float duty = FF_DUTY_PER_RPM * target_rpm + PI_KP * error + PI_KI * integral;

  if (fabsf(duty) < MIN_START_DUTY) {
    duty = copysignf(MIN_START_DUTY, target_rpm);
  }

  duty = clampf(duty, static_cast<float>(-PWM_MAX_DUTY),
                static_cast<float>(PWM_MAX_DUTY));
  return static_cast<int>(duty);
}

static void handle_velocity_command(float left_rpm, float right_rpm) {
  if (isnan(left_rpm) || isnan(right_rpm)) {
    return;
  }

  if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) != pdTRUE) return;

  g_target_left_rpm = clampf(left_rpm, -MAX_ABS_RPM, MAX_ABS_RPM);
  g_target_right_rpm = clampf(right_rpm, -MAX_ABS_RPM, MAX_ABS_RPM);
  g_last_command_ms = millis();
  g_watchdog_tripped = false;

  xSemaphoreGive(g_state_mutex);
}

static void handle_frame(const esp32_protocol::FrameParser &parser) {
  using namespace esp32_protocol;

  if (parser.type() != PacketType::VelocityCmd ||
      parser.length() != sizeof(VelocityCmd)) {
    return;
  }

  VelocityCmd cmd;
  std::memcpy(&cmd, parser.payload(), sizeof(cmd));
  handle_velocity_command(cmd.left_rpm, cmd.right_rpm);
}

static void serial_task_poll_rx() {
  static esp32_protocol::FrameParser parser;

  while (Serial.available() > 0) {
    const auto result = parser.feed(static_cast<uint8_t>(Serial.read()));
    if (result == esp32_protocol::FrameParser::Result::FrameReady) {
      handle_frame(parser);
      parser.reset();
    } else if (result == esp32_protocol::FrameParser::Result::InvalidFrame) {
      parser.reset();
    }
  }
}

static void run_control_step(uint32_t now_ms) {
  const float dt_s = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

  const int32_t left_ticks = read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  const int32_t right_ticks = read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

  const int32_t delta_left = left_ticks - g_last_left_ticks;
  const int32_t delta_right = right_ticks - g_last_right_ticks;
  g_last_left_ticks = left_ticks;
  g_last_right_ticks = right_ticks;

  g_measured_left_rpm =
      (static_cast<float>(delta_left) / TICKS_PER_REVOLUTION) / dt_s * 60.0f;
  g_measured_right_rpm =
      (static_cast<float>(delta_right) / TICKS_PER_REVOLUTION) / dt_s * 60.0f;

  if (estop_is_pressed()) {
    stop_motors_locked(false);
    return;
  }

  if ((now_ms - g_last_command_ms) > WATCHDOG_TIMEOUT_MS) {
    if (!g_watchdog_tripped) {
      stop_motors_locked(false);
      g_watchdog_tripped = true;
    }
    return;
  }

  const int left_duty =
      pi_to_pwm(g_target_left_rpm, g_measured_left_rpm, g_integral_left, dt_s);
  const int right_duty = pi_to_pwm(g_target_right_rpm, g_measured_right_rpm,
                                   g_integral_right, dt_s);

  set_motor_pwm(LEFT_PWM_CHANNEL, LEFT_DIR_PIN, left_duty);
  set_motor_pwm(RIGHT_PWM_CHANNEL, RIGHT_DIR_PIN, right_duty);
}

static void control_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    const uint32_t now_ms = millis();

    if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
      run_control_step(now_ms);
      xSemaphoreGive(g_state_mutex);
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

static void publish_encoder_report() {
  const int32_t left_ticks = read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  const int32_t right_ticks = read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

  esp32_protocol::EncoderReport report;
  report.left_ticks = left_ticks;
  report.right_ticks = right_ticks;
  report.timestamp_us = micros();

  for (int i = 0; i < 3; ++i) {
    report.accel[i] = 0;
    report.gyro[i] = 0;
  }

  if (g_mpu6050_initialized) {
    int16_t accel[3] = {0, 0, 0};
    int16_t gyro[3] = {0, 0, 0};
    if (read_mpu6050_raw(accel, gyro)) {
      for (int i = 0; i < 3; ++i) {
        report.accel[i] = accel[i];
        report.gyro[i] = gyro[i];
      }
    }
  }

  uint8_t frame[sizeof(report) + 6];
  const uint16_t frame_len = esp32_protocol::frame_pack(
      esp32_protocol::PacketType::EncoderReport, &report, sizeof(report), frame);

  if (g_serial_mutex != nullptr &&
      xSemaphoreTake(g_serial_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    Serial.write(frame, frame_len);
    xSemaphoreGive(g_serial_mutex);
  }
}

static void report_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    publish_encoder_report();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(REPORT_PERIOD_MS));
  }
}

static void serial_task(void *parameter) {
  (void)parameter;

  for (;;) {
    serial_task_poll_rx();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void start_rtos_tasks() {
  g_state_mutex = xSemaphoreCreateMutex();
  g_serial_mutex = xSemaphoreCreateMutex();

  if (g_state_mutex == nullptr || g_serial_mutex == nullptr) return;

  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK_WORDS,
                          nullptr, CONTROL_TASK_PRIORITY,
                          &g_control_task_handle, 1);
  xTaskCreatePinnedToCore(report_task, "report", REPORT_TASK_STACK_WORDS,
                          nullptr, REPORT_TASK_PRIORITY,
                          &g_report_task_handle, 1);
  xTaskCreatePinnedToCore(serial_task, "serial", SERIAL_TASK_STACK_WORDS,
                          nullptr, SERIAL_TASK_PRIORITY,
                          &g_serial_task_handle, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);
  ledcWrite(LEFT_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);

  if (ESTOP_PIN >= 0) {
    pinMode(ESTOP_PIN, INPUT_PULLUP);
  }

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  g_left_encoder.attachFullQuad(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
  g_right_encoder.attachFullQuad(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
  g_left_encoder.clearCount();
  g_right_encoder.clearCount();

  g_last_command_ms = millis();
  g_last_left_ticks = read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  g_last_right_ticks = read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

  g_mpu6050_initialized = init_mpu6050();

  start_rtos_tasks();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
