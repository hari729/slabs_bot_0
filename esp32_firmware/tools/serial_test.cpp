// PC-side motor/encoder test for the ESP32 firmware.
// Build: g++ -std=c++17 -O2 -pthread tools/serial_test.cpp -o serial_test
// Run:   ./serial_test [--port /dev/ttyUSB0] [--rpm 60] [--secs 3] [--tpr 2060] [--tol 15]
// Lift the wheels off the ground first.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../include/protocol.h"

using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_abort{false};
static int g_fd = -1;

static std::mutex g_mu;
static long g_ticks[2] = {0, 0};
static unsigned long g_packets = 0;

static void send_frame(esp32_protocol::PacketType type, const void *payload, uint8_t len) {
  if (g_fd < 0) return;
  uint8_t frame[64];
  const uint16_t n = esp32_protocol::frame_pack(type, payload, len, frame);
  (void)!write(g_fd, frame, n);
}

static void send_velocity(float left_rpm, float right_rpm) {
  esp32_protocol::VelocityCmd cmd{left_rpm, right_rpm};
  send_frame(esp32_protocol::PacketType::VelocityCmd, &cmd, sizeof(cmd));
}

static void on_sigint(int) { g_abort = true; }

static bool open_port(const char *path) {
  g_fd = open(path, O_RDWR | O_NOCTTY);
  if (g_fd < 0) return false;
  termios tio{};
  tcgetattr(g_fd, &tio);
  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  tcsetattr(g_fd, TCSANOW, &tio);
  tcflush(g_fd, TCIOFLUSH);
  return true;
}

static void reader_thread() {
  esp32_protocol::FrameParser parser;
  uint8_t c;
  while (!g_abort) {
    const ssize_t n = read(g_fd, &c, 1);
    if (n <= 0) continue;

    const auto result = parser.feed(c);
    if (result != esp32_protocol::FrameParser::Result::FrameReady) {
      if (result == esp32_protocol::FrameParser::Result::InvalidFrame) {
        parser.reset();
      }
      continue;
    }

    if (parser.type() == esp32_protocol::PacketType::EncoderReport &&
        parser.length() == sizeof(esp32_protocol::EncoderReport)) {
      esp32_protocol::EncoderReport report;
      std::memcpy(&report, parser.payload(), sizeof(report));
      std::lock_guard<std::mutex> lk(g_mu);
      g_ticks[0] = report.left_ticks;
      g_ticks[1] = report.right_ticks;
      g_packets++;
    }
    parser.reset();
  }
}

struct Snap {
  long t[2];
  unsigned long pk;
  Clock::time_point at;
};

static Snap snap() {
  std::lock_guard<std::mutex> lk(g_mu);
  return {{g_ticks[0], g_ticks[1]}, g_packets, Clock::now()};
}

static void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

struct Result {
  std::string name;
  double rpm[2];
  long delta[2];
  double rate_hz;
  bool pass;
};

static std::vector<Result> g_results;
static double g_rpm, g_secs, g_tpr, g_tol;

static Result run_phase(const std::string &name, double l, double r) {
  const double tgt[2] = {l, r};

  const auto start = Clock::now();
  Snap a{};
  bool a_taken = false;
  while (!g_abort) {
    const double el = std::chrono::duration<double>(Clock::now() - start).count();
    if (el >= g_secs) break;
    send_velocity(static_cast<float>(l), static_cast<float>(r));
    if (!a_taken && el >= 1.0) {
      a = snap();
      a_taken = true;
    }
    sleep_ms(100);
  }
  const Snap b = snap();
  send_velocity(0.0f, 0.0f);

  Result res{name, {0, 0}, {0, 0}, 0, true};
  if (!a_taken) {
    res.pass = false;
    printf("%-22s no measurement window\n", name.c_str());
    return res;
  }

  const double dt = std::chrono::duration<double>(b.at - a.at).count();
  res.rate_hz = (b.pk - a.pk) / dt;
  if (res.rate_hz < 40 || res.rate_hz > 60) res.pass = false;

  for (int i = 0; i < 2; i++) {
    res.delta[i] = b.t[i] - a.t[i];
    res.rpm[i] = (res.delta[i] / g_tpr) / dt * 60.0;
    if (tgt[i] != 0.0) {
      const bool sign_ok = (res.rpm[i] > 0) == (tgt[i] > 0);
      const double err = std::fabs(res.rpm[i] - tgt[i]) / std::fabs(tgt[i]) * 100.0;
      if (!sign_ok || err > g_tol) res.pass = false;
    } else if (std::labs(res.delta[i]) > g_tpr * 0.05) {
      res.pass = false;
    }
  }
  printf("%-22s L: tgt %6.1f meas %7.1f (%+7ld) | R: tgt %6.1f meas %7.1f (%+7ld) | %4.0f Hz | %s\n",
         name.c_str(), l, res.rpm[0], res.delta[0], r, res.rpm[1], res.delta[1],
         res.rate_hz, res.pass ? "PASS" : "FAIL");
  fflush(stdout);
  return res;
}

static void stop_and_wait() {
  send_velocity(0.0f, 0.0f);
  sleep_ms(800);
}

static void watchdog_test() {
  printf("Watchdog test...\n");
  const double v = g_rpm;
  for (int i = 0; i < 20 && !g_abort; i++) {
    send_velocity(static_cast<float>(v), static_cast<float>(v));
    sleep_ms(100);
  }
  sleep_ms(900);
  const Snap a = snap();
  sleep_ms(500);
  const Snap b = snap();
  const long moved = std::labs(b.t[0] - a.t[0]) + std::labs(b.t[1] - a.t[1]);
  const bool ok = moved < g_tpr * 0.02;
  printf("%-22s ticks moved after timeout: %ld | %s\n", "watchdog", moved,
         ok ? "PASS" : "FAIL");
  g_results.push_back({"watchdog", {0, 0}, {0, 0}, 0, ok});
  stop_and_wait();
}

int main(int argc, char **argv) {
  const char *port = "/dev/ttyUSB0";
  g_rpm = 60;
  g_secs = 3;
  g_tpr = 2060;
  g_tol = 15;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--port")) port = argv[i + 1];
    else if (!strcmp(argv[i], "--rpm")) g_rpm = atof(argv[i + 1]);
    else if (!strcmp(argv[i], "--secs")) g_secs = atof(argv[i + 1]);
    else if (!strcmp(argv[i], "--tpr")) g_tpr = atof(argv[i + 1]);
    else if (!strcmp(argv[i], "--tol")) g_tol = atof(argv[i + 1]);
  }
  if (g_secs < 2) g_secs = 2;

  signal(SIGINT, on_sigint);
  if (!open_port(port)) {
    perror(port);
    return 1;
  }
  std::thread rd(reader_thread);

  sleep_ms(2500);
  send_velocity(0.0f, 0.0f);
  sleep_ms(500);
  if (snap().pk == 0) {
    printf("No encoder packets received on %s\n", port);
    g_abort = true;
    rd.join();
    return 1;
  }
  printf("Port %s | %.0f RPM | %.1f s/phase | tpr %.0f | tol %.0f%%\n\n",
         port, g_rpm, g_secs, g_tpr, g_tol);

  struct Case { const char *name; double l, r; };
  const Case cases[] = {
      {"left forward", g_rpm, 0},    {"left reverse", -g_rpm, 0},
      {"right forward", 0, g_rpm},   {"right reverse", 0, -g_rpm},
      {"both forward", g_rpm, g_rpm}, {"both reverse", -g_rpm, -g_rpm},
      {"spin (L+ R-)", g_rpm, -g_rpm},
  };
  for (const Case &c : cases) {
    if (g_abort) break;
    g_results.push_back(run_phase(c.name, c.l, c.r));
    stop_and_wait();
  }
  if (!g_abort) watchdog_test();

  send_velocity(0.0f, 0.0f);
  g_abort = true;
  rd.join();
  close(g_fd);

  int fails = 0;
  printf("\n=== Summary ===\n");
  for (const Result &r : g_results) {
    printf("%-22s %s\n", r.name.c_str(), r.pass ? "PASS" : "FAIL");
    if (!r.pass) fails++;
  }
  printf("%d/%zu passed\n", (int)g_results.size() - fails, g_results.size());
  return fails ? 2 : 0;
}
