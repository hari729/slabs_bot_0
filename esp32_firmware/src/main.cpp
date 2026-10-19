// =============================================================================
// ESP32 differential-drive firmware
//
// Responsibilities (and nothing else):
//   1. Read left/right quadrature encoders via the ESP32 PCNT peripheral
//      (wrapped by the ESP32Encoder library — not polled GPIO).
//   2. Drive two motors with PWM + direction pins.
//   3. Run a 100 Hz PI velocity loop that tracks RPM setpoints.
//   4. Speak a line-based serial protocol with the Jetson.
//   5. Watchdog: if no valid command for 500 ms, force motors to stop.
//
// FreeRTOS layout (three dedicated tasks):
//   - control_task : 100 Hz PI loop + watchdog + e-stop (highest priority)
//   - report_task  : 50 Hz encoder line TX
//   - serial_task  : non-blocking RX line parser + debug status TX
//
// Protocol (115200 8N1):
//   TX (Jetson -> ESP32):  "V <left_rpm> <right_rpm>\n"
//   RX (ESP32  -> Jetson): "E <left_ticks> <right_ticks> <timestamp_us>\n"
//
// Extra commands for minicom debugging (not used by ROS):
//   "S\n"   immediate stop
//   "?\n"   print a human-readable status line
// =============================================================================

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <stdarg.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// -----------------------------------------------------------------------------
// Pin map — change these to match your wiring.
//
// Motor driver assumed: PWM + DIR (TB6612, BTS7960 in DIR mode, etc.).
//   DIR HIGH = forward, DIR LOW = reverse, PWM duty = speed.
// If your driver uses IN1/IN2 instead, replace set_motor_pwm() accordingly.
// -----------------------------------------------------------------------------
static const int LEFT_PWM_PIN = 19;
static const int LEFT_DIR_PIN = 18;
static const int RIGHT_PWM_PIN = 26;
static const int RIGHT_DIR_PIN = 25;

// Quadrature encoder channels. Pins 34 and 35 are input-only on ESP32
// (good for encoders, but they have no internal pull-up — add 10k to 3.3 V).
static const int RIGHT_ENC_A_PIN = 32;
static const int RIGHT_ENC_B_PIN = 33;
static const int LEFT_ENC_A_PIN = 34;
static const int LEFT_ENC_B_PIN = 35;

// Optional hardware e-stop. Wire a normally-closed switch to ground.
// Set to -1 to disable this input.
static const int ESTOP_PIN = 4;

// One wheel is physically mirrored. Negate that side so "forward" ticks
// increase on both joints. Flip this if your robot drives backwards.
static const int LEFT_ENCODER_SIGN = -1;
static const int RIGHT_ENCODER_SIGN = 1;

// -----------------------------------------------------------------------------
// Mechanical / encoder constants.
// ticks_per_revolution MUST match the ros2_control hardware parameter.
// Measure it: rotate a wheel exactly 10 turns, divide tick delta by 10.
// -----------------------------------------------------------------------------
static const float TICKS_PER_REVOLUTION = 1399.2f;

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------
static const uint32_t CONTROL_PERIOD_MS = 10; // 100 Hz PI loop
static const uint32_t REPORT_PERIOD_MS = 20;  // 50 Hz encoder reports
static const uint32_t WATCHDOG_TIMEOUT_MS = 500;

// -----------------------------------------------------------------------------
// FreeRTOS task configuration
// -----------------------------------------------------------------------------
static const uint32_t CONTROL_TASK_STACK_WORDS = 4096;
static const uint32_t REPORT_TASK_STACK_WORDS = 3072;
static const uint32_t SERIAL_TASK_STACK_WORDS = 4096;

// Control runs at the highest priority so motor safety timing stays tight.
static const UBaseType_t CONTROL_TASK_PRIORITY = 3;
static const UBaseType_t REPORT_TASK_PRIORITY = 2;
static const UBaseType_t SERIAL_TASK_PRIORITY = 2;

// How long a task waits to acquire a mutex before giving up (milliseconds).
static const TickType_t MUTEX_TIMEOUT_TICKS = pdMS_TO_TICKS(50);

// -----------------------------------------------------------------------------
// PWM (LEDC)
// -----------------------------------------------------------------------------
static const int PWM_FREQUENCY_HZ = 20000;
static const int PWM_RESOLUTION_BITS = 10; // 0 .. 1023
static const int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
static const int LEFT_PWM_CHANNEL = 0;
static const int RIGHT_PWM_CHANNEL = 1;

// -----------------------------------------------------------------------------
// Velocity PI gains (start conservative; tune on the floor).
// Units: PWM duty per RPM of error.
// -----------------------------------------------------------------------------
static const float FF_DUTY_PER_RPM = 7.522;
static const float MIN_START_DUTY = 98.0;
static const float PI_KP = 2.051;
static const float PI_KI = 20.515;
static const float PI_INTEGRAL_LIMIT =
    100.0f; // rpm*s -> up to 800 duty at Ki=8
static const float MAX_ABS_RPM = 150.0f;

// -----------------------------------------------------------------------------
// Runtime state (shared between RTOS tasks — always guard with g_state_mutex)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// FreeRTOS handles
// -----------------------------------------------------------------------------
static SemaphoreHandle_t g_state_mutex = nullptr;
static SemaphoreHandle_t g_serial_mutex = nullptr;

static TaskHandle_t g_control_task_handle = nullptr;
static TaskHandle_t g_report_task_handle = nullptr;
static TaskHandle_t g_serial_task_handle = nullptr;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// Clamp a float into [min_value, max_value].
static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

// Read the signed encoder count for one wheel.
static int32_t read_encoder_ticks(ESP32Encoder &encoder, int sign) {
  return static_cast<int32_t>(encoder.getCount()) * sign;
}

// Drive one motor. duty is in [-PWM_MAX_DUTY, +PWM_MAX_DUTY].
// Positive duty = forward (DIR high). Negative = reverse (DIR low).
static void set_motor_pwm(int pwm_channel, int dir_pin, int duty) {
  duty = constrain(duty, -PWM_MAX_DUTY, PWM_MAX_DUTY);

  if (duty >= 0) {
    digitalWrite(dir_pin, LOW);   // IN2 low
    ledcWrite(pwm_channel, duty); // IN1 = PWM -> forward
  } else {
    digitalWrite(dir_pin, HIGH); // IN2 high
    ledcWrite(pwm_channel,
              PWM_MAX_DUTY + duty); // IN1 = inverted PWM -> reverse
  }
}
// Stop both motors and reset PI state.
// Caller must already hold g_state_mutex, OR pass take_mutex = true.
static void stop_motors_locked(bool take_mutex) {
  if (take_mutex) {
    if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) != pdTRUE) {
      // If we cannot get the lock, still cut PWM hardware immediately.
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
  if (ESTOP_PIN < 0) {
    return false;
  }
  // Active LOW with internal pull-up.
  return digitalRead(ESTOP_PIN) == LOW;
}

// Convert RPM error into a PWM duty using a PI controller.
// static int pi_to_pwm(float target_rpm, float measured_rpm, float &integral,
//                      float dt_s) {
//   const float error = target_rpm - measured_rpm;
//   integral += error * dt_s;
//   integral = clampf(integral, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
//
//   float duty = (PI_KP * error) + (PI_KI * integral);
//   duty = clampf(duty, static_cast<float>(-PWM_MAX_DUTY),
//                 static_cast<float>(PWM_MAX_DUTY));
//   return static_cast<int>(duty);
// }
static int pi_to_pwm(float target_rpm, float measured_rpm, float &integral,
                     float dt_s) {
  if (fabsf(target_rpm) < 1.0f) { // target ~0 -> stop and reset
    integral = 0.0f;
    return 0;
  }

  const float error = target_rpm - measured_rpm;
  integral += error * dt_s;
  integral = clampf(integral, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);

  float duty = FF_DUTY_PER_RPM * target_rpm + PI_KP * error + PI_KI * integral;

  // Make sure we are at least at breakaway duty, in the commanded direction.
  if (fabsf(duty) < MIN_START_DUTY) {
    duty = copysignf(MIN_START_DUTY, target_rpm);
  }

  duty = clampf(duty, static_cast<float>(-PWM_MAX_DUTY),
                static_cast<float>(PWM_MAX_DUTY));
  return static_cast<int>(duty);
}
// Thread-safe Serial.printf — keeps encoder lines and status lines from
// interleaving.
static void serial_printf_locked(const char *format, ...) {
  if (g_serial_mutex == nullptr) {
    return;
  }

  if (xSemaphoreTake(g_serial_mutex, MUTEX_TIMEOUT_TICKS) != pdTRUE) {
    return;
  }

  va_list args;
  va_start(args, format);
  char buffer[160];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);

  xSemaphoreGive(g_serial_mutex);
}

// -----------------------------------------------------------------------------
// Serial protocol (handled entirely inside serial_task)
// -----------------------------------------------------------------------------

static void handle_velocity_command(float left_rpm, float right_rpm) {
  if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) != pdTRUE) {
    return;
  }

  g_target_left_rpm = clampf(left_rpm, -MAX_ABS_RPM, MAX_ABS_RPM);
  g_target_right_rpm = clampf(right_rpm, -MAX_ABS_RPM, MAX_ABS_RPM);
  g_last_command_ms = millis();
  g_watchdog_tripped = false;

  xSemaphoreGive(g_state_mutex);
}

static void handle_serial_line(const String &line) {
  if (line.length() == 0) {
    return;
  }

  if (line.charAt(0) == 'V' || line.charAt(0) == 'v') {
    float left_rpm = 0.0f;
    float right_rpm = 0.0f;
    // sscanf on the C string after skipping the command letter.
    const int parsed =
        sscanf(line.c_str() + 1, " %f %f", &left_rpm, &right_rpm);
    if (parsed == 2) {
      handle_velocity_command(left_rpm, right_rpm);
    }
    return;
  }

  if (line.charAt(0) == 'S' || line.charAt(0) == 's') {
    stop_motors_locked(true);

    if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
      g_last_command_ms = millis();
      g_watchdog_tripped = false;
      xSemaphoreGive(g_state_mutex);
    }
    return;
  }

  if (line.charAt(0) == '?') {
    float measured_left = 0.0f;
    float measured_right = 0.0f;
    float target_left = 0.0f;
    float target_right = 0.0f;
    bool watchdog = false;
    const int32_t left_ticks =
        read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
    const int32_t right_ticks =
        read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

    if (xSemaphoreTake(g_state_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
      measured_left = g_measured_left_rpm;
      measured_right = g_measured_right_rpm;
      target_left = g_target_left_rpm;
      target_right = g_target_right_rpm;
      watchdog = g_watchdog_tripped;
      xSemaphoreGive(g_state_mutex);
    }

    serial_printf_locked("STATUS left_rpm=%.2f right_rpm=%.2f tgt_l=%.2f "
                         "tgt_r=%.2f ticks_l=%ld ticks_r=%ld wd=%d\n",
                         measured_left, measured_right, target_left,
                         target_right, static_cast<long>(left_ticks),
                         static_cast<long>(right_ticks), watchdog ? 1 : 0);
  }
}

// Drain the UART RX buffer and assemble newline-terminated command lines.
static void serial_task_poll_rx() {
  static String serial_line;

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      serial_line.trim();
      handle_serial_line(serial_line);
      serial_line = "";
    } else {
      // Guard against a flood of garbage filling RAM.
      if (serial_line.length() < 80) {
        serial_line += incoming;
      } else {
        serial_line = "";
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Control loop (100 Hz) — runs in its own FreeRTOS task
// -----------------------------------------------------------------------------

static void run_control_step(uint32_t now_ms) {
  const float dt_s = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

  const int32_t left_ticks =
      read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  const int32_t right_ticks =
      read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

  const int32_t delta_left = left_ticks - g_last_left_ticks;
  const int32_t delta_right = right_ticks - g_last_right_ticks;
  g_last_left_ticks = left_ticks;
  g_last_right_ticks = right_ticks;

  // RPM = (ticks / ticks_per_rev) / dt * 60
  g_measured_left_rpm =
      (static_cast<float>(delta_left) / TICKS_PER_REVOLUTION) / dt_s * 60.0f;
  g_measured_right_rpm =
      (static_cast<float>(delta_right) / TICKS_PER_REVOLUTION) / dt_s * 60.0f;

  // Safety: e-stop pin forces a hard stop (checked outside the mutex too).
  if (estop_is_pressed()) {
    stop_motors_locked(false);
    return;
  }

  // Watchdog: no valid command recently -> stop once and stay stopped.
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

  // vTaskDelayUntil gives a stable 100 Hz cadence (better than millis()
  // polling).
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

// -----------------------------------------------------------------------------
// Encoder report loop (50 Hz) — runs in its own FreeRTOS task
// -----------------------------------------------------------------------------

static void publish_encoder_report() {
  const int32_t left_ticks =
      read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  const int32_t right_ticks =
      read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);
  const uint32_t timestamp_us = micros();

  serial_printf_locked("E %ld %ld %lu\n", static_cast<long>(left_ticks),
                       static_cast<long>(right_ticks),
                       static_cast<unsigned long>(timestamp_us));
}

static void report_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    publish_encoder_report();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(REPORT_PERIOD_MS));
  }
}

// -----------------------------------------------------------------------------
// Serial RX task — dedicated UART polling loop
// -----------------------------------------------------------------------------

static void serial_task(void *parameter) {
  (void)parameter;

  for (;;) {
    serial_task_poll_rx();
    // Short yield so other same-priority tasks (report) still get CPU time.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -----------------------------------------------------------------------------
// FreeRTOS startup
// -----------------------------------------------------------------------------

static void start_rtos_tasks() {
  g_state_mutex = xSemaphoreCreateMutex();
  g_serial_mutex = xSemaphoreCreateMutex();

  if (g_state_mutex == nullptr || g_serial_mutex == nullptr) {
    Serial.println("# FATAL: could not create FreeRTOS mutexes");
    return;
  }

  // Control task: pin to core 1 (Arduino loop/core 1 convention on ESP32).
  BaseType_t control_ok = xTaskCreatePinnedToCore(
      control_task, "control", CONTROL_TASK_STACK_WORDS, nullptr,
      CONTROL_TASK_PRIORITY, &g_control_task_handle, 1);

  BaseType_t report_ok = xTaskCreatePinnedToCore(
      report_task, "report", REPORT_TASK_STACK_WORDS, nullptr,
      REPORT_TASK_PRIORITY, &g_report_task_handle, 1);

  BaseType_t serial_ok = xTaskCreatePinnedToCore(
      serial_task, "serial", SERIAL_TASK_STACK_WORDS, nullptr,
      SERIAL_TASK_PRIORITY, &g_serial_task_handle, 0);

  if (control_ok != pdPASS || report_ok != pdPASS || serial_ok != pdPASS) {
    Serial.println("# FATAL: could not create one or more FreeRTOS tasks");
  }
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

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

  // Full-quadrature decode uses the ESP32 PCNT hardware (not GPIO polling).
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  g_left_encoder.attachFullQuad(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
  g_right_encoder.attachFullQuad(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
  g_left_encoder.clearCount();
  g_right_encoder.clearCount();

  g_last_command_ms = millis();
  g_last_left_ticks = read_encoder_ticks(g_left_encoder, LEFT_ENCODER_SIGN);
  g_last_right_ticks = read_encoder_ticks(g_right_encoder, RIGHT_ENCODER_SIGN);

  start_rtos_tasks();

  Serial.println("# ESP32 motor firmware ready (FreeRTOS). Send: V <left_rpm> "
                 "<right_rpm>");
}

void loop() {
  // All real-time work lives in FreeRTOS tasks. Keep loop() idle.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
