#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../include/protocol.h"

static int g_fd = -1;
static volatile bool g_abort = false;

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

int main(int argc, char **argv) {
  const char *port = "/dev/ttyUSB0";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--port")) port = argv[i + 1];
  }

  signal(SIGINT, on_sigint);

  if (!open_port(port)) {
    perror(port);
    return 1;
  }

  printf("Listening on %s\n", port);

  esp32_protocol::FrameParser parser;
  uint8_t c;
  while (!g_abort) {
    if (read(g_fd, &c, 1) <= 0) continue;

    const auto result = parser.feed(c);
    if (result == esp32_protocol::FrameParser::Result::FrameReady) {
      if (parser.type() == esp32_protocol::PacketType::EncoderReport &&
          parser.length() == sizeof(esp32_protocol::EncoderReport)) {
        esp32_protocol::EncoderReport report;
        std::memcpy(&report, parser.payload(), sizeof(report));
        printf("L=%10ld  R=%10ld  t=%10u us\n",
               static_cast<long>(report.left_ticks),
               static_cast<long>(report.right_ticks),
               static_cast<unsigned int>(report.timestamp_us));
        fflush(stdout);
      }
      parser.reset();
    } else if (result == esp32_protocol::FrameParser::Result::InvalidFrame) {
      parser.reset();
    }
  }

  close(g_fd);
  return 0;
}
