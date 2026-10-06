#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace simp_controller {

// Index of the tracking point that governs `now_ns`: the latest point whose
// time (stamp + index * period) is not after now, clamped to the array.
// Integer arithmetic keeps exact grid times on their own point.
inline std::size_t tracking_index(std::int64_t now_ns, std::int64_t stamp_ns,
                                  std::int64_t period_ns, std::size_t size) {
  if (size == 0) throw std::invalid_argument("empty tracking trajectory");
  if (period_ns <= 0) throw std::invalid_argument("tracking period must be positive");
  if (now_ns <= stamp_ns) return 0;
  const auto index = static_cast<std::uint64_t>((now_ns - stamp_ns) / period_ns);
  return static_cast<std::size_t>(std::min<std::uint64_t>(index, size - 1));
}

}  // namespace simp_controller
