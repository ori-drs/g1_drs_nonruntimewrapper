#pragma once

#include <cstdint>

namespace g1_state_bridge
{

// Keeps messages at no more than rate_hz on average, judged by message stamps.
// rate_hz <= 0 keeps every message.
class RateLimiter
{
public:
  explicit RateLimiter(double rate_hz = 0.0)
  : period_ns_(rate_hz > 0.0 ? static_cast<int64_t>(1e9 / rate_hz) : 0) {}

  bool allow(int64_t stamp_ns)
  {
    if (period_ns_ == 0) {
      return true;
    }
    if (started_ && stamp_ns < next_ns_) {
      return false;
    }
    if (!started_ || stamp_ns - next_ns_ >= period_ns_) {
      next_ns_ = stamp_ns + period_ns_;   // (re)start after a gap
    } else {
      next_ns_ += period_ns_;             // keep a steady cadence
    }
    started_ = true;
    return true;
  }

private:
  int64_t period_ns_;
  int64_t next_ns_ = 0;
  bool started_ = false;
};

}  // namespace g1_state_bridge
