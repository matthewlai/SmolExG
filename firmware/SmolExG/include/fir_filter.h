#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <cstdint>
#include <cstring>

// FIR filter state buffer
struct FIRFilter {
  const FilterCoeffType* coeffs = nullptr;
  int num_taps;
  int32_t *history = nullptr;
  int write_idx;

  FIRFilter(const FilterCoeffType* coeffs, int num_taps)
      : coeffs(coeffs), num_taps(num_taps), history(new int32_t[num_taps]), write_idx(0) {
    std::fill(history, history + num_taps, 0);
  }
  
  ~FIRFilter() {
    delete[] history;
  }
  
  float IRAM_ATTR apply(int32_t sample) {    
    // Insert sample into circular buffer
    history[write_idx] = sample;
    write_idx = (write_idx + 1) % num_taps;
    
    // Compute FIR output (split loop to avoid modulus)
    int64_t output = 0;
    
    // First part: from write_idx to end of buffer
    for (int i = 0; i < num_taps - write_idx; ++i) {
      output += static_cast<int64_t>(history[write_idx + i]) * coeffs[i];
    }
    
    // Second part: from beginning of buffer to write_idx
    for (int i = num_taps - write_idx; i < num_taps; ++i) {
      output += static_cast<int64_t>(history[i - num_taps + write_idx]) * coeffs[i];
    }

    return static_cast<float>(output) / (1 << FILTER_FIXED_POINT_BITS);
  }
};

#endif // FIR_FILTER_H
