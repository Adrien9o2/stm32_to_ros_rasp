#pragma once
#include <cstdint>
namespace SerialID
{
	inline constexpr uint8_t MSG_ACK = 0xFF;
	inline constexpr uint8_t MSG_START = 0xFE;
	inline constexpr uint8_t MSG_NO_START = 0xFD;
	inline constexpr uint8_t MSG_NO_ACK = 0xFC;
	inline constexpr uint8_t MSG_NO_ID = 0xFB;
	inline constexpr uint8_t MSG_NO_SIZE = 0xFA;
	inline constexpr uint8_t MSG_PRINT = 0x00;
	inline constexpr uint8_t MSG_MOTOR_SPEEDS = 0x01;
};