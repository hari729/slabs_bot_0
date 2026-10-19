// =============================================================================
// pid_autotune.cpp — standalone ESP32 motor identification + PI auto-tuner
//
// LIFT THE WHEELS OFF THE GROUND before running. Motors spin up to full duty.
//
// What it does, for each motor (left, then right), in both directions:
//   1. Breakaway duty : smallest duty that makes the wheel turn.
//   2. Duty sweep     : steady-state RPM at 5 duties -> fit  duty = d0 + k*rpm
//                       (k = feedforward gain, d0 = offset / deadband, plus
//                        the maximum RPM at full duty).
//   3. Step response  : duty step (forward only) sampled at 100 Hz ->
//                       plant gain K [rpm/duty], time constant tau, dead time L
//                       (Smith two-point method), then IMC/lambda PI tuning:
//                          Kp = tau / (K * (lambda + L)),   Ki = Kp / tau
//
// Usage:
//   1. Flash this INSTEAD of the main firmware (separate PlatformIO env or
//      temporarily replace main.cpp).
//   2. Open the serial monitor at 115200, type  g  + Enter to start.
//      Type  x  at any time to abort (motors stop).
//   3. Copy the printed constants into the main firmware.
//
// Keep PWM_FREQ / PWM_BITS / pins / ENC_SIGN identical to the main firmware,
// and run it on the final battery / supply — the results depend on voltage.
// =============================================================================

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

// ---- Hardware (must match the main firmware) --------------------------------
static const int PWM_PIN[2] = {19, 26}; // [0]=left, [1]=right  (L298N IN2)
static const int DIR_PIN[2] = {18, 25}; //                      (L298N IN1)
static const int ENC_A[2] = {34, 32};
static const int ENC_B[2] = {35, 33};
static const int ENC_SIGN[2] = {-1, 1};
static const int PWM_CH[2] = {0, 1};
static const char *NAME[2] = {"LEFT", "RIGHT"};

static const float TICKS_PER_REV = 1399.2f;
static const int PWM_FREQ = 20000;
static const int PWM_BITS = 10;
static const int PWM_MAX = (1 << PWM_BITS) - 1;

// Closed-loop speed relative to the plant's own time constant.
// 1.0 = closed loop about as fast as the motor (conservative).
// 0.5 = about twice as fast (more aggressive, may overshoot).
static const float LAMBDA_FACTOR = 1.0f;

// ---- State ------------------------------------------------------------------
static ESP32Encoder enc[2];

struct Fit { // duty = d0 + k * rpm  (per direction)
  bool ok = false;
  bool wrong_sign = false;
  float d0 = 0, k = 0, maxrpm = 0;
};
struct StepFit {
  bool ok = false;
  float K = 0, tau = 0, L = 0;
};
struct MotorResult {
  float breakaway[2] = {-1, -1}; // [0]=forward, [1]=reverse
  Fit fit[2];
  StepFit step;
};
static MotorResult res[2];

// ---- Low-level helpers ------------------------------------------------------
static void set_motor(int m, int duty) {
  duty = constrain(duty, -PWM_MAX, PWM_MAX);
  if (duty >= 0) {
    digitalWrite(DIR_PIN[m], LOW);
    ledcWrite(PWM_CH[m], duty);
  } else {
    digitalWrite(DIR_PIN[m], HIGH);
    ledcWrite(PWM_CH[m], PWM_MAX + duty); // inverted PWM for reverse
  }
}

static void stop_all() {
  set_motor(0, 0);
  set_motor(1, 0);
}

static void abort_now() {
  stop_all();
  Serial.println("# ABORTED by user");
  while (true)
    delay(1000);
}

// delay() that can be aborted with 'x'
static void wait_ms(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (Serial.available() && Serial.read() == 'x')
      abort_now();
    delay(1);
  }
}

static int32_t ticks(int m) {
  return static_cast<int32_t>(enc[m].getCount()) * ENC_SIGN[m];
}

// Average RPM over a window (signed, includes ENC_SIGN).
static float measure_rpm(int m, uint32_t ms) {
  const int32_t t0 = ticks(m);
  const uint32_t u0 = micros();
  wait_ms(ms);
  const int32_t t1 = ticks(m);
  const uint32_t u1 = micros();
  const float dt = static_cast<float>(static_cast<uint32_t>(u1 - u0)) / 1e6f;
  return (static_cast<float>(t1 - t0) / TICKS_PER_REV) / dt * 60.0f;
}

// ---- Test 1: breakaway duty -------------------------------------------------
static float find_breakaway(int m, int dir) {
  for (int duty = 80; duty <= PWM_MAX; duty += 10) {
    set_motor(m, dir * duty);
    const float rpm = fabsf(measure_rpm(m, 150));
    if (rpm > 3.0f) {
      stop_all();
      wait_ms(800);
      return static_cast<float>(duty);
    }
  }
  stop_all();
  wait_ms(800);
  return -1.0f;
}

// ---- Test 2: steady-state duty sweep + linear fit ---------------------------
static Fit run_sweep(int m, int dir) {
  static const float FRACTIONS[] = {0.45f, 0.60f, 0.75f, 0.90f, 1.00f};
  const int N = sizeof(FRACTIONS) / sizeof(FRACTIONS[0]);

  Fit f;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int n = 0;

  for (int i = 0; i < N; i++) {
    const int duty = static_cast<int>(FRACTIONS[i] * PWM_MAX);
    set_motor(m, dir * duty);
    wait_ms(1500);                               // settle
    const float rpm = measure_rpm(m, 800) * dir; // positive = as commanded
    Serial.printf("#   duty %c%4d -> %7.2f rpm\n", dir > 0 ? '+' : '-', duty,
                  rpm);

    if (rpm < -2.0f)
      f.wrong_sign = true;
    if (rpm > 2.0f) {
      sx += rpm;
      sy += duty;
      sxx += static_cast<double>(rpm) * rpm;
      sxy += static_cast<double>(rpm) * duty;
      n++;
    }
    if (i == N - 1)
      f.maxrpm = rpm;
  }
  stop_all();
  wait_ms(1000);

  const double den = n * sxx - sx * sx;
  if (n >= 3 && fabs(den) > 1e-6) {
    f.k = static_cast<float>((n * sxy - sx * sy) / den);
    f.d0 = static_cast<float>((sy - f.k * sx) / n);
    f.ok = f.k > 0.0f;
  }
  return f;
}

// ---- Test 3: step response -> K, tau, L -------------------------------------
static const int STEP_N = 250; // 2.5 s at 100 Hz
static const int W = 5;        // 50 ms RPM window
static int32_t st_ticks[STEP_N + 1];
static uint32_t st_us[STEP_N + 1];
static float st_y[STEP_N + 1]; // normalised response 0..1
static float st_t[STEP_N + 1]; // time of each y sample (s since step)

static float window_rpm(int i) {
  const float dt =
      static_cast<float>(static_cast<uint32_t>(st_us[i] - st_us[i - W])) / 1e6f;
  return (static_cast<float>(st_ticks[i] - st_ticks[i - W]) / TICKS_PER_REV) /
         dt * 60.0f;
}

static float crossing(float level) {
  for (int i = W + 1; i <= STEP_N; i++) {
    if (st_y[i - 1] < level && st_y[i] >= level) {
      const float frac = (level - st_y[i - 1]) / (st_y[i] - st_y[i - 1]);
      return st_t[i - 1] + frac * (st_t[i] - st_t[i - 1]);
    }
  }
  return -1.0f;
}

static StepFit run_step(int m, int d_lo, int d_hi) {
  StepFit s;

  set_motor(m, d_lo);
  wait_ms(2000);
  const float rpm_lo = measure_rpm(m, 500);

  st_ticks[0] = ticks(m);
  st_us[0] = micros();
  set_motor(m, d_hi); // <-- the step
  uint32_t next = st_us[0];
  for (int i = 1; i <= STEP_N; i++) {
    next += 10000; // 10 ms
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    st_ticks[i] = ticks(m);
    st_us[i] = micros();
  }
  stop_all();
  wait_ms(1000);

  float fin = 0;
  for (int i = STEP_N - 49; i <= STEP_N; i++)
    fin += window_rpm(i);
  fin /= 50.0f;
  const float delta = fin - rpm_lo;
  Serial.printf("#   step %d -> %d duty: %.1f -> %.1f rpm\n", d_lo, d_hi,
                rpm_lo, fin);
  if (delta < 5.0f) {
    Serial.println("#   step produced too little speed change");
    return s;
  }

  for (int i = W; i <= STEP_N; i++) {
    st_y[i] = (window_rpm(i) - rpm_lo) / delta;
    // Window average is centred half a window in the past.
    st_t[i] = (static_cast<float>(st_us[i] - st_us[0]) +
               static_cast<float>(st_us[i - W] - st_us[0])) /
              2e6f;
  }

  const float t28 = crossing(0.283f);
  const float t63 = crossing(0.632f);
  Serial.printf("#   t28=%.3f s  t63=%.3f s\n", t28, t63);
  if (t28 < 0 || t63 <= t28) {
    Serial.println(
        "#   could not extract time constant (plant too fast or noisy)");
    return s;
  }

  s.tau = 1.5f * (t63 - t28);
  s.L = fmaxf(t63 - s.tau, 0.0f);
  s.K = delta / static_cast<float>(d_hi - d_lo); // rpm per duty
  s.ok = s.tau > 0.005f && s.K > 0.0f;
  return s;
}

// ---- Per-motor routine ------------------------------------------------------
static void tune_motor(int m) {
  Serial.printf("# ---------- %s motor ----------\n", NAME[m]);
  stop_all();
  wait_ms(500);

  for (int d = 0; d < 2; d++) {
    const int dir = (d == 0) ? 1 : -1;
    Serial.printf("# %s %s: breakaway ramp...\n", NAME[m],
                  d == 0 ? "forward" : "reverse");
    res[m].breakaway[d] = find_breakaway(m, dir);
    Serial.printf("#   breakaway duty = %.0f\n", res[m].breakaway[d]);
    Serial.printf("# %s %s: duty sweep...\n", NAME[m],
                  d == 0 ? "forward" : "reverse");
    res[m].fit[d] = run_sweep(m, dir);
    if (res[m].fit[d].wrong_sign) {
      Serial.printf(
          "# !!! %s: rpm sign is OPPOSITE to the commanded direction -> "
          "flip ENC_SIGN for this wheel (or swap the motor wires)\n",
          NAME[m]);
    }
  }

  const float bk = res[m].breakaway[0];
  if (bk > 0) {
    float d_lo = fmaxf(0.5f * PWM_MAX, 1.25f * bk);
    d_lo = fminf(d_lo, 0.7f * PWM_MAX);
    const float d_hi = fminf(d_lo + 0.25f * PWM_MAX, 0.95f * PWM_MAX);
    Serial.printf("# %s: step response...\n", NAME[m]);
    res[m].step = run_step(m, static_cast<int>(d_lo), static_cast<int>(d_hi));
  } else {
    Serial.printf("# %s: motor never started, skipping step test\n", NAME[m]);
  }
}

// ---- Report -----------------------------------------------------------------
struct Gains {
  bool ok = false;
  float k = 0, d0 = 0, kp = 0, ki = 0, bk = 0;
};

static Gains compute_gains(int m) {
  Gains g;
  const MotorResult &r = res[m];

  float k = 0, d0 = 0, bk = 0;
  int c = 0;
  for (int d = 0; d < 2; d++) {
    if (r.fit[d].ok) {
      k += r.fit[d].k;
      d0 += r.fit[d].d0;
      c++;
    }
    bk = fmaxf(bk, r.breakaway[d]);
  }
  if (c == 0)
    return g;
  if (c == 2) {
    const float ratio = r.fit[0].k / r.fit[1].k;
    if (ratio > 1.25f || ratio < 0.8f) {
      Serial.printf("// WARNING %s: forward/reverse gain differ by %.0f%% "
                    "(check encoder wiring / pull-ups)\n",
                    NAME[m], fabsf(ratio - 1.0f) * 100.0f);
    }
  }
  g.k = k / c;
  g.d0 = d0 / c;
  g.bk = bk;

  float K = r.step.K, tau = r.step.tau, L = r.step.L;
  if (!r.step.ok) {
    // Plant is faster than a 100 Hz loop / this encoder can resolve.
    // Use the sweep slope for K and assume tau ~ 30 ms (t63 was ~30 ms).
    K = 1.0f / g.k;
    tau = 0.03f;
    L = 0.01f;
    Serial.printf("// NOTE %s: step fit failed, assuming tau = 30 ms\n",
                  NAME[m]);
  }

  // A 100 Hz loop with ~4 rpm tick quantisation cannot exploit a 30 ms plant,
  // so floor the closed-loop time constant and integral time at 100 ms.
  const float lambda = fmaxf(LAMBDA_FACTOR * fmaxf(tau, L), 0.10f);
  const float ti = fmaxf(tau, 0.10f);
  g.kp = tau / (K * (lambda + L));
  g.ki = g.kp / ti;
  g.ok = true;
  return g;
}

static void print_summary() {
  Serial.println();
  Serial.println("// ================= RESULTS =================");

  float min_max = 1e9f;
  for (int m = 0; m < 2; m++)
    for (int d = 0; d < 2; d++)
      if (res[m].fit[d].ok)
        min_max = fminf(min_max, res[m].fit[d].maxrpm);

  Gains g[2];
  for (int m = 0; m < 2; m++) {
    g[m] = compute_gains(m);
    Serial.printf(
        "// %s: breakaway fwd %.0f / rev %.0f | max rpm fwd %.1f / rev %.1f | "
        "plant K=%.4f rpm/duty tau=%.3f s L=%.3f s\n",
        NAME[m], res[m].breakaway[0], res[m].breakaway[1], res[m].fit[0].maxrpm,
        res[m].fit[1].maxrpm, res[m].step.K, res[m].step.tau, res[m].step.L);
    if (!g[m].ok)
      Serial.printf("// %s: FAILED, no usable constants (see log above)\n",
                    NAME[m]);
  }

  if (min_max > 1e8f) {
    Serial.println(
        "// No valid sweep data at all. Check supply, wiring, encoder.");
    return;
  }

  Serial.println();
  Serial.println("// ---- paste into main firmware ----");
  Serial.printf("static const float MAX_ABS_RPM = %.1f;   // 90%% of slowest "
                "full-duty speed\n",
                0.9f * min_max);

  for (int m = 0; m < 2; m++) {
    if (!g[m].ok)
      continue;
    Serial.printf("static const float FF_DUTY_PER_RPM_%s = %.3f;\n", NAME[m],
                  g[m].k);
    Serial.printf("static const float FF_OFFSET_DUTY_%s   = %.1f;\n", NAME[m],
                  g[m].d0);
    Serial.printf("static const float PI_KP_%s = %.3f;\n", NAME[m], g[m].kp);
    Serial.printf("static const float PI_KI_%s = %.3f;\n", NAME[m], g[m].ki);
    Serial.printf("// %s breakaway (start) duty: %.0f\n", NAME[m], g[m].bk);
  }

  if (g[0].ok && g[1].ok) {
    Serial.println();
    Serial.println("// ---- OR one shared set (average of both sides) ----");
    Serial.printf("static const float FF_DUTY_PER_RPM = %.3f;\n",
                  0.5f * (g[0].k + g[1].k));
    Serial.printf("static const float FF_OFFSET_DUTY  = %.1f;\n",
                  0.5f * (g[0].d0 + g[1].d0));
    Serial.printf("static const float PI_KP = %.3f;\n",
                  0.5f * (g[0].kp + g[1].kp));
    Serial.printf("static const float PI_KI = %.3f;\n",
                  0.5f * (g[0].ki + g[1].ki));
  }
  Serial.println("// =============================================");
}

// ---- Arduino entry points ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  for (int m = 0; m < 2; m++) {
    pinMode(DIR_PIN[m], OUTPUT);
    ledcSetup(PWM_CH[m], PWM_FREQ, PWM_BITS);
    ledcAttachPin(PWM_PIN[m], PWM_CH[m]);
  }
  stop_all();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc[0].attachFullQuad(ENC_A[0], ENC_B[0]);
  enc[1].attachFullQuad(ENC_A[1], ENC_B[1]);
  enc[0].clearCount();
  enc[1].clearCount();

  delay(1000);
  Serial.println("# PID auto-tune. LIFT THE WHEELS. Type 'g' + Enter to start, "
                 "'x' to abort.");
  while (true) {
    if (Serial.available() && Serial.read() == 'g')
      break;
    delay(10);
  }
  Serial.println("# Starting in 3 s...");
  delay(3000);

  tune_motor(0);
  tune_motor(1);
  stop_all();
  print_summary();
}

void loop() { delay(1000); }
