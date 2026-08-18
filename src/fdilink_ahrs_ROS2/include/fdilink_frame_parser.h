#ifndef FDILINK_FRAME_PARSER_H_
#define FDILINK_FRAME_PARSER_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace FDILink
{

constexpr std::uint8_t FRAME_HEAD = 0xfc;
constexpr std::uint8_t FRAME_END = 0xfd;
constexpr std::uint8_t TYPE_IMU = 0x40;
constexpr std::uint8_t TYPE_AHRS = 0x41;
constexpr std::uint8_t TYPE_INSGPS = 0x42;
constexpr std::uint8_t TYPE_GEODETIC_POS = 0x5c;
constexpr std::uint8_t TYPE_GROUND = 0xf0;
constexpr std::uint8_t TYPE_GROUND_EXTENDED = 0x50;

constexpr std::uint8_t IMU_LEN = 0x38;
constexpr std::uint8_t AHRS_LEN = 0x30;
constexpr std::uint8_t INSGPS_LEN = 0x48;
constexpr std::uint8_t GEODETIC_POS_LEN = 0x20;

constexpr std::size_t FDILINK_HEADER_SIZE = 7;
constexpr std::size_t FDILINK_MAX_PAYLOAD_SIZE = 255;
constexpr std::size_t FDILINK_MAX_FRAME_SIZE =
    FDILINK_HEADER_SIZE + FDILINK_MAX_PAYLOAD_SIZE + 1;

struct ValidatedFrame
{
  std::uint8_t type = 0;
  std::uint8_t serial_number = 0;
  std::uint8_t payload_size = 0;
  std::size_t frame_size = 0;
  std::array<std::uint8_t, FDILINK_MAX_FRAME_SIZE> bytes{};
};

enum class FrameParserEvent
{
  None,
  FrameReady,
  Rejected,
};

class FrameParser
{
public:
  FrameParser() = default;

  FrameParserEvent consume(std::uint8_t byte, ValidatedFrame& output);
  void reset() noexcept;
  bool hasPartialFrame() const noexcept;

private:
  static bool isSupportedType(std::uint8_t type) noexcept;
  static bool isIgnoredType(std::uint8_t type) noexcept;
  static bool lengthMatchesType(
      std::uint8_t type, std::uint8_t payload_size) noexcept;

  FrameParserEvent reject(std::uint8_t current_byte) noexcept;

  std::array<std::uint8_t, FDILINK_MAX_FRAME_SIZE> buffer_{};
  std::size_t size_ = 0;
  std::size_t expected_size_ = 0;
};

}  // namespace FDILink

#endif  // FDILINK_FRAME_PARSER_H_
