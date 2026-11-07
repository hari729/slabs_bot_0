#ifndef MEARM_ARDUINO_PROTOCOL_H
#define MEARM_ARDUINO_PROTOCOL_H

/**
 * @file protocol.h
 * @brief Shared binary protocol between the ROS 2 arm driver and the Arduino.
 *
 * Wire format (little-endian, packed):
 *   [0xAA][0x55][type u8][len u8][payload...][crc16 u16]
 *
 * The CRC-16/CCITT-FALSE covers type, len and payload bytes.
 */

#include <stdint.h>
#include <string.h>

namespace mearm_protocol
{

static const uint8_t SYNC0 = 0xAA;
static const uint8_t SYNC1 = 0x55;

enum PacketType : uint8_t {
  ServoCmdPacket = 0x10,
};

#pragma pack(push, 1)

struct ServoCmd {
  float joint_0;  // base: continuous-rotation speed, -1.0 .. +1.0
  float joint_1;  // shoulder: physical servo angle in radians, 0.5708 .. pi
  float joint_2;  // elbow: physical servo angle in radians, 0 .. pi/2
  float joint_3;  // claw: physical servo angle in radians, 1.3963=closed .. 1.6581=open
};

#pragma pack(pop)

static_assert(sizeof(ServoCmd) == 16, "ServoCmd packing mismatch");

/**
 * @brief CRC-16/CCITT-FALSE.
 */
inline uint16_t crc16(const uint8_t * data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/**
 * @brief Pack a struct into a transmit buffer.
 * @param type Packet type.
 * @param payload Pointer to the packed struct.
 * @param payload_len Size of the packed struct.
 * @param out Output buffer, must hold at least payload_len + 6 bytes.
 * @return Total number of bytes written.
 */
inline uint16_t frame_pack(
  PacketType type, const void * payload, uint8_t payload_len, uint8_t * out)
{
  out[0] = SYNC0;
  out[1] = SYNC1;
  out[2] = static_cast<uint8_t>(type);
  out[3] = payload_len;
  if (payload_len > 0) {
    memcpy(&out[4], payload, payload_len);
  }
  const uint16_t crc = crc16(&out[2], payload_len + 2);
  out[4 + payload_len] = static_cast<uint8_t>(crc >> 8);
  out[5 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
  return static_cast<uint16_t>(payload_len + 6);
}

/**
 * @brief Stateful byte parser for the framed protocol.
 */
class FrameParser
{
public:
  enum Result {
    NeedMore,
    FrameReady,
    InvalidFrame,
  };

  void reset()
  {
    state_ = State::Sync0;
    payload_len_ = 0;
    received_ = 0;
  }

  Result feed(uint8_t byte)
  {
    switch (state_) {
      case State::Sync0:
        if (byte == SYNC0) {
          state_ = State::Sync1;
        }
        return Result::NeedMore;

      case State::Sync1:
        if (byte == SYNC1) {
          state_ = State::Type;
        } else {
          state_ = (byte == SYNC0) ? State::Sync1 : State::Sync0;
        }
        return Result::NeedMore;

      case State::Type:
        type_ = byte;
        state_ = State::Length;
        return Result::NeedMore;

      case State::Length:
        payload_len_ = byte;
        received_ = 0;
        if (payload_len_ > kMaxPayload) {
          state_ = State::Sync0;
          return Result::InvalidFrame;
        }
        state_ = (payload_len_ == 0) ? State::CrcHigh : State::Payload;
        return Result::NeedMore;

      case State::Payload:
        payload_[received_++] = byte;
        if (received_ >= payload_len_) {
          state_ = State::CrcHigh;
        }
        return Result::NeedMore;

      case State::CrcHigh:
        crc_ = static_cast<uint16_t>(byte) << 8;
        state_ = State::CrcLow;
        return Result::NeedMore;

      case State::CrcLow:
        crc_ |= byte;
        {
          uint8_t crc_input[2 + kMaxPayload];
          crc_input[0] = type_;
          crc_input[1] = payload_len_;
          memcpy(&crc_input[2], payload_, payload_len_);
          const bool ok = (crc16(crc_input, payload_len_ + 2) == crc_);
          state_ = State::Sync0;
          return ok ? Result::FrameReady : Result::InvalidFrame;
        }
    }
    return Result::NeedMore;
  }

  PacketType type() const { return static_cast<PacketType>(type_); }
  uint8_t length() const { return payload_len_; }
  const uint8_t * payload() const { return payload_; }

private:
  static const uint8_t kMaxPayload = 32;

  enum class State {
    Sync0,
    Sync1,
    Type,
    Length,
    Payload,
    CrcHigh,
    CrcLow,
  };

  State state_{State::Sync0};
  uint8_t type_{0};
  uint8_t payload_len_{0};
  uint8_t received_{0};
  uint8_t payload_[kMaxPayload];
  uint16_t crc_{0};
};

}  // namespace mearm_protocol

#endif  // MEARM_ARDUINO_PROTOCOL_H
