import argparse
import threading
import serial
from serial.tools import list_ports


def read_serial(ser):
    """Continuously print incoming ESP32 messages."""
    while ser.is_open:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"\rESP32: {line}")
        except (serial.SerialException, OSError):
            break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            # Opening the port may reset the ESP32.
            print(f"Connected to {args.port} at {args.baud} baud.")
            print("Commands: V 100 100 | V -100 100 | S | ? | quit")

            reader = threading.Thread(target=read_serial, args=(ser,), daemon=True)
            reader.start()

            while True:
                try:
                    command = input("\n> ").strip()
                except (EOFError, KeyboardInterrupt):
                    break

                if command.lower() in ("quit", "exit"):
                    break

                if command:
                    ser.write((command + "\n").encode("ascii"))
                    ser.flush()

    except serial.SerialException as exc:
        print(f"Serial error: {exc}")


if __name__ == "__main__":
    main()
