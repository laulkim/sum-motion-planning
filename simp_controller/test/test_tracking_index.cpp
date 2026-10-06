#include "simp_controller/tracking_index.hpp"

#include <iostream>
#include <stdexcept>

using simp_controller::tracking_index;

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
}  // namespace

int main() {
  try {
    constexpr std::int64_t stamp = 12'300'000'000;
    constexpr std::int64_t period = 10'000'000;
    require(tracking_index(stamp, stamp, period, 100) == 0, "stamp did not map to the first point");
    // 12.3437 s is 43.7 ms after the stamp: the 12.340 s point governs.
    require(tracking_index(12'343'700'000, stamp, period, 100) == 4, "index was not the floor");
    require(tracking_index(stamp + 4 * period, stamp, period, 100) == 4,
            "exact grid time did not use its own point");
    require(tracking_index(stamp + 4 * period - 1, stamp, period, 100) == 3,
            "time just before a grid point used the next point");
    require(tracking_index(stamp + 1'000 * period, stamp, period, 100) == 99,
            "time past the end did not hold the last point");
    require(tracking_index(stamp - period, stamp, period, 100) == 0,
            "time before the stamp did not use the first point");
    bool rejected = false;
    try {
      tracking_index(stamp, stamp, period, 0);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "empty trajectory was accepted");
    std::cout << "all tracking index tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
