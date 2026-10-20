#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

namespace ego_planner
{
// Only recovery replans reset the cooldown; ordinary replans do not.
class TrackingErrorTrigger
{
public:
  TrackingErrorTrigger(double threshold = 0.30, double confirmation = 0.20,
                       double interval = 1.0)
      : threshold_(threshold), confirmation_(confirmation), interval_(interval)
  {
    if (!std::isfinite(threshold) || threshold <= 0 ||
        !std::isfinite(confirmation) || confirmation <= 0 ||
        !std::isfinite(interval) || interval <= 0)
      throw std::invalid_argument("tracking error threshold, confirmation and interval must be finite and positive");
  }

  void reset(double now)
  {
    above_since_ = -1.0;
    last_recovery_ = last_sample_ = now;
  }

  bool update(double error, double now)
  {
    if (!std::isfinite(error) || !std::isfinite(now))
    {
      above_since_ = -1.0;
      return false;
    }
    if (now < last_sample_)
      reset(now);
    last_sample_ = now;
    if (error <= threshold_)
    {
      above_since_ = -1.0;
      return false;
    }
    if (above_since_ < 0)
      above_since_ = now;
    if (now - above_since_ < confirmation_ || now - last_recovery_ < interval_)
      return false;
    reset(now);
    return true;
  }

private:
  double threshold_, confirmation_, interval_;
  double above_since_ = -1.0;
  double last_sample_ = -std::numeric_limits<double>::infinity();
  double last_recovery_ = -std::numeric_limits<double>::infinity();
};
} // namespace ego_planner
