#pragma once

#include <sys/resource.h>
#include <time.h>

#include <chrono>

namespace transport {

// Process CPU time so far (summed over all threads), as a chrono duration. std::chrono has no
// CPU-time clock of its own, so the reading comes from the POSIX per-process CPU clock rather
// than a chrono clock. CLOCK_PROCESS_CPUTIME_ID yields nanosecond resolution, unlike
// std::clock()'s CLOCKS_PER_SEC (microsecond) scaling.
inline std::chrono::nanoseconds process_cpu_now() {
    timespec ts{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec);
}

// Process-wide peak resident set size so far, in megabytes. ru_maxrss is a monotonic
// high-water mark on Linux, so sampling it at successive phase boundaries yields the peak
// RSS reached by the end of each phase.
inline double peak_rss_mb() {
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
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
