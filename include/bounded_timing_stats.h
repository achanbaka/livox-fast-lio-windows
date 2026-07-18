#ifndef BOUNDED_TIMING_STATS_H
#define BOUNDED_TIMING_STATS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

struct TimingWindowSummary {
    uint64_t total_samples = 0;
    size_t window_samples = 0;
    double average_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
};

// A fixed-size rolling sample window. Recording is O(1), allocates no memory,
// and never grows with process uptime. The relatively more expensive sort is
// deliberately deferred to summary generation (normally shutdown/tests).
template <size_t Capacity = 1024>
class BoundedTimingWindow {
    static_assert(Capacity > 0, "timing window capacity must be positive");

public:
    static constexpr size_t capacity() noexcept { return Capacity; }

    void record(double milliseconds) noexcept {
        if (!std::isfinite(milliseconds) || milliseconds < 0.0) return;
        samples_[next_] = milliseconds;
        next_ = (next_ + 1) % Capacity;
        if (size_ < Capacity) ++size_;
        ++total_samples_;
    }

    TimingWindowSummary summary() const {
        TimingWindowSummary result;
        result.total_samples = total_samples_;
        result.window_samples = size_;
        if (size_ == 0) return result;

        std::array<double, Capacity> sorted{};
        double sum = 0.0;
        for (size_t i = 0; i < size_; ++i) {
            sorted[i] = samples_[i];
            sum += sorted[i];
        }
        std::sort(sorted.begin(), sorted.begin() + size_);
        result.average_ms = sum / static_cast<double>(size_);
        result.p95_ms = percentile(sorted, size_, 0.95);
        result.p99_ms = percentile(sorted, size_, 0.99);
        return result;
    }

private:
    static double percentile(const std::array<double, Capacity>& sorted,
                             size_t size, double quantile) noexcept {
        const size_t rank = static_cast<size_t>(
            std::ceil(quantile * static_cast<double>(size)));
        return sorted[std::max<size_t>(1, rank) - 1];
    }

    std::array<double, Capacity> samples_{};
    size_t next_ = 0;
    size_t size_ = 0;
    uint64_t total_samples_ = 0;
};

#endif
