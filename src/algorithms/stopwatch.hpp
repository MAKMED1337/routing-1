#pragma once

#include <chrono>
#include <ctime>

namespace transport {

inline std::chrono::nanoseconds process_cpu_now() {
    using Ticks = std::chrono::duration<std::clock_t, std::ratio<1, CLOCKS_PER_SEC>>;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Ticks(std::clock()));
}

struct Stopwatch {
    std::chrono::steady_clock::time_point wall_start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds cpu_start = process_cpu_now();

    [[nodiscard]] std::chrono::nanoseconds wall_elapsed() const {
        return std::chrono::steady_clock::now() - wall_start;
    }
    [[nodiscard]] std::chrono::nanoseconds cpu_elapsed() const { return process_cpu_now() - cpu_start; }
};

inline double to_seconds(std::chrono::nanoseconds d) { return std::chrono::duration<double>(d).count(); }

} // namespace transport
