#pragma once

namespace manual_flight_assistant {

namespace joypad {

namespace axes {
//static constexpr uint32_t kX = 4;
//static constexpr uint32_t kY = 3;
//static constexpr uint32_t kZ = 1;
//static constexpr uint32_t kYaw = 0;
static constexpr uint32_t kX = 1;
static constexpr uint32_t kY = 3;
static constexpr uint32_t kZ = 4;
static constexpr uint32_t kYaw = 0;
}  // namespace axes

namespace buttons {
static constexpr uint32_t kGreen = 3;
static constexpr uint32_t kRed = 1;
static constexpr uint32_t kBlue = 0;//2
static constexpr uint32_t kYellow = 2;

static constexpr uint32_t kLb = 8;//4
static constexpr uint32_t kRb = 9;//5
}  // namespace buttons

}  // namespace joypad

}  // namespace manual_flight_assistant
