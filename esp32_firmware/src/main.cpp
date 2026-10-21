#include <Arduino.h>
#include <ESP32Encoder.h>
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

  start_rtos_tasks();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
