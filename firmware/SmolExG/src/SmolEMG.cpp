#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <SPI.h>

#include <U8g2lib.h>

#include "adc.h"
#include "button_manager.h"
#include "pffft.h"
#include "filters.h"
#include "fir_filter.h"

// High resolution mode -
// 0x0 - 32 kSPS (requires modifying ADC read to support, since the chip will switch to reporting 16-bit values)
// 0x1 - 16 kSPS
// 0x2 - 8 kSPS
// 0x3 - 4 kSPS
// 0x4 - 2 kSPS
// 0x5 - 1 kSPS
// 0x6 - 500 SPS
constexpr uint8_t DATA_RATE_SETTING = 0x4;
constexpr uint32_t DATA_RATE_SPS = 2000;

const FilterCoeffType* FIR_FILTER = FILTER_20_500;
const int FIR_FILTER_TAPS = sizeof(FILTER_20_500) / sizeof(*FILTER_20_500);

constexpr int PWDN_PIN = 1;
constexpr int BUTTON_1_PIN = 3;
constexpr int BUTTON_2_PIN = 2;
constexpr int VBATT_DIV_2_PIN = 0;

constexpr int ADS1298_CS_PIN = 20;
constexpr int ADS1298_DO_PIN = 10;
constexpr int ADS1298_DIN_PIN = 6;
constexpr int ADS1298_CLK_PIN = 21;

constexpr uint32_t IDLE_SLEEP_TIME_MS = 300000;

// Screen mode enumeration
enum class ScreenMode {
  TIME_DOMAIN,
  FREQUENCY_DOMAIN
};

// Structure to store min/max values for each time bin.
struct BinData {
  int32_t min_val;
  int32_t max_val;
};

constexpr int PLOT_X = 0;
constexpr int PLOT_Y = 8;
constexpr int PLOT_W = 128;
constexpr int PLOT_H = 64 - PLOT_Y;
constexpr float TIME_PLOT_WINDOW_S = 1.0f;

// Scale options in tenths of nanovolts (10nV units)
// Displayed as: 100 µV = 10,000 (10nV), 300 µV = 30,000, 1000 µV = 100,000
constexpr int SCALE_OPTIONS[] = {10000, 30000, 100000};
constexpr int NUM_SCALES = sizeof(SCALE_OPTIONS) / sizeof(SCALE_OPTIONS[0]);

// FFT configuration
constexpr int FFT_SIZE = 256;
constexpr int FFT_NUM_BINS = FFT_SIZE / 2;  // Real FFT gives us N/2 frequency bins
constexpr int FFT_UPDATE_INTERVAL_MS = 200;
constexpr int FFT_UPDATE_SAMPLES = (DATA_RATE_SPS * FFT_UPDATE_INTERVAL_MS) / 1000;  // 400 samples at 2000 Hz

// Caclulated params.
constexpr int TIME_PLOT_DOWNSAMPLE = static_cast<int>(DATA_RATE_SPS / PLOT_W * TIME_PLOT_WINDOW_S);
constexpr int TIME_PLOT_MID_SCALE = PLOT_Y + PLOT_H / 2;

// Precomputed Hamming window coefficients
float fft_window[FFT_SIZE];

void compute_hamming_window() {
  for (int i = 0; i < FFT_SIZE; ++i) {
    fft_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
  }
}

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(/*rotation=*/U8G2_R0, /*reset=*/U8X8_PIN_NONE, /*clk=*/5, /*data=*/4);

void go_to_sleep() {
  // Power gate ADC and display.
  display.setPowerSave(true);
  digitalWrite(PWDN_PIN, LOW);
  delay(100);
  esp_deep_sleep_start();
}

void draw_fft_spectrum(float *fft_output, int fft_size, int max_val_tnv) {
  // Draw FFT magnitude spectrum with reference magnitude based on current scale
  // fft_output contains interleaved complex numbers (real, imag, real, imag, ...)
  // We need to compute magnitude as sqrt(real^2 + imag^2)
  
  // Use a reference magnitude consistent with the time-domain display scale
  // For a sine wave of amplitude max_val_tnv (in 10nV units), the FFT magnitude is ~max_val_tnv * FFT_SIZE / 2
  const float FIXED_SCALE = max_val_tnv * (FFT_SIZE / 2.0f);
  
  // Draw frequency bins as vertical bars
  for (int x = 0; x < PLOT_W && (x + 1) <= FFT_NUM_BINS; ++x) {
    float real_part = fft_output[2 * x];
    float imag_part = fft_output[2 * x + 1];
    float magnitude = sqrtf(real_part * real_part + imag_part * imag_part);
    
    // Use log scale for better visualization with reference matching time-domain
    float log_mag = 20.0f * log10f(magnitude / FIXED_SCALE + 1e-6f) + 80.0f;  // Normalize to 0-80dB
    float normalized = std::max(0.0f, std::min(1.0f, log_mag / 80.0f));
    
    int bar_height = static_cast<int>(normalized * PLOT_H);
    bar_height = std::min(bar_height, PLOT_H);
    
    // Draw vertical bar at this x position
    for (int y = 0; y < bar_height; ++y) {
      display.drawPixel(x, PLOT_Y + PLOT_H - 1 - y);
    }
  }
}

void itoa_fixed_width(uint32_t x, char* buf, int width) {
  uint32_t max_val = 1;
  for (int i = 0; i < width; ++i) {
    max_val *= 10;
  }
  max_val -= 1;
  if (x > max_val) {
    x = max_val;
  }
  itoa(x, buf, 10);
  int current_width = strlen(buf);
  if (current_width < width) {
    int leading_zeros = width - current_width;
    for (int i = 0; i < leading_zeros; ++i) {
      buf[i] = '0';
    }
    itoa(x, buf + leading_zeros, 10);
  }
}

uint32_t batt_v_mv() {
  // We have a 1M:2M divider on vbatt.
  // This is quite high, and ESP32-C3 input pins have max
  // 50nA leakage, which is significant in this case.
  // Actually, from my one test chip, the value seems to be
  // around 116nA, though there may be leakage elsewhere.
  constexpr float EST_LEAKAGE = 116e-9f;
  float vbatt_raw = analogReadMilliVolts(VBATT_DIV_2_PIN) / 1000.0f;
  float leakage_r_eq = vbatt_raw / EST_LEAKAGE;
  float low_r_eq = 1.0f / (1.0f / 1e6f + 1.0f / leakage_r_eq);
  float vbatt_est = (low_r_eq + 1e6f) / low_r_eq * vbatt_raw;
  return static_cast<uint32_t>(vbatt_est * 1000.0);
}

void setup() {
  // First enable USB serial for debugging.
  Serial.begin();

  // Set up pin modes and instantiate debouncer.
  pinMode(PWDN_PIN, OUTPUT);
  pinMode(VBATT_DIV_2_PIN, INPUT);
  pinMode(ADS1298_DRDY_PIN, INPUT);
  ButtonManager btn1(BUTTON_1_PIN);
  ButtonManager btn2(BUTTON_2_PIN);

  // Enable power to the ADC and OLED display.
  digitalWrite(PWDN_PIN, HIGH);

  // Wait a bit for the ADC and display to power up.
  // ADS1298 needs 2^18 internal clocks (t_por), which
  // is around 130ms. No idea how much time the display
  // needs, but this seems to work.
  delay(200);

  // Set up wake up sources for if/when we go to sleep.
  // Pressing either button wakes the chip up.
  esp_deep_sleep_enable_gpio_wakeup(1 << BUTTON_1_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_enable_gpio_wakeup(1 << BUTTON_2_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);

  // Initialize the display.
  display.begin();
  display.setFont(u8g2_font_t0_12_mf);

  // Set up ADC for battery voltage monitoring.
  analogSetAttenuation(ADC_11db); // 0-2500mV.

  // Initialize ADS1298 ADC.
  bool adc_init = ads1298_init(
    DATA_RATE_SETTING,
    ADS1298_CS_PIN,
    ADS1298_DO_PIN,
    ADS1298_DIN_PIN,
    ADS1298_CLK_PIN,
    ADS1298_DRDY_PIN
  );

  uint32_t last_activity_time = millis();

  int32_t sample_buf[8];

  // Min/max ADC values for each time bin.
  BinData time_plot_rolling_window[PLOT_W];
  std::fill(
    std::begin(time_plot_rolling_window),
    std::end(time_plot_rolling_window),
    BinData{0, 0});
  int time_plot_rolling_window_start = 0;

  // FFT setup
  ScreenMode current_screen = ScreenMode::TIME_DOMAIN;
  int scale_index = 0;  // Start with 100uV scale
  int current_max_val_tnv = SCALE_OPTIONS[scale_index];
  float current_time_plot_scale = static_cast<float>(PLOT_H / 2) / current_max_val_tnv;
  
  PFFFT_Setup *fft_setup = pffft_new_setup(FFT_SIZE, PFFFT_REAL);
  compute_hamming_window();
  
  // Allocate SIMD-aligned buffers for FFT
  float *fft_input = (float *)pffft_aligned_malloc(FFT_SIZE * sizeof(float));
  float *fft_output = (float *)pffft_aligned_malloc(FFT_SIZE * sizeof(float));
  float *fft_work = (float *)pffft_aligned_malloc(FFT_SIZE * sizeof(float));
  
  // Rolling FFT circular buffer
  float *fft_circular_buffer = (float *)pffft_aligned_malloc(FFT_SIZE * sizeof(float));
  std::fill(fft_circular_buffer, fft_circular_buffer + FFT_SIZE, 0.0f);
  
  int fft_buffer_idx = 0;  // Current write position in circular buffer
  int fft_samples_since_update = 0;  // Samples collected since last FFT update
  bool fft_has_valid_data = false;  // Whether we've filled the buffer once

  // Time-domain accumulation for downsampling
  int32_t time_plot_min_val = INT32_MAX;
  int32_t time_plot_max_val = INT32_MIN;
  int time_plot_accumulator = 0;
  
  // Initialize FIR filter for 10 Hz high-pass filtering
  FIRFilter fir_filter(FIR_FILTER, FIR_FILTER_TAPS);

  for (;;) {
    auto btn1_state = btn1.Update();
    auto btn2_state = btn2.Update();

    display.clearBuffer();
    uint32_t vbatt_mv = batt_v_mv();

    if (btn1_state != ButtonManager::State::NONE ||
      btn2_state != ButtonManager::State::NONE) {
      last_activity_time = millis();
    }

    // BUTTON_1: Cycle through scales (100uV, 300uV, 1000uV)
    if (btn1_state == ButtonManager::State::SHORT) {
      scale_index = (scale_index + 1) % NUM_SCALES;
      current_max_val_tnv = SCALE_OPTIONS[scale_index];
      current_time_plot_scale = static_cast<float>(PLOT_H / 2) / current_max_val_tnv;
    }

    // BUTTON_2: Switch between time-domain and frequency-domain displays
    if (btn2_state == ButtonManager::State::SHORT) {
      current_screen = (current_screen == ScreenMode::TIME_DOMAIN) 
                       ? ScreenMode::FREQUENCY_DOMAIN 
                       : ScreenMode::TIME_DOMAIN;
    }

    // Draw battery level.
    char buf[16];
    strcpy(buf, "B=_.__V");
    buf[2] = '0' + (vbatt_mv / 1000) % 10;
    itoa_fixed_width((vbatt_mv % 1000) / 10, buf + 4, 2);
    strcat(buf, "V");
    display.drawStr(0, 8, buf);

    // Display scale information (convert 10nV units to µV for display)
    char scale_buf[16];
    int scale_uv = current_max_val_tnv / 100;
    sprintf(scale_buf, "S=%duV", scale_uv);
    display.drawStr(70, 8, scale_buf);

    if (!adc_init) {
      display.drawStr(0, 26, "ADC init failed");
      display.sendBuffer();
      break;
    }
 
    uint32_t idle_time_ms = millis() - last_activity_time;

    if (idle_time_ms > IDLE_SLEEP_TIME_MS) {
      go_to_sleep();
    }
    while (adc_data_available() > 0) {
      int32_t raw_sample = read_adc_buffer();
      
      // Apply bandpass FIR filter (20-500 Hz)
      auto start = micros();
      float sample_f = fir_filter.apply(raw_sample);
      int32_t sample = static_cast<int32_t>(sample_f);
      
      // Add to rolling FFT circular buffer
      fft_circular_buffer[fft_buffer_idx] = sample_f;
      fft_buffer_idx = (fft_buffer_idx + 1) % FFT_SIZE;
      fft_samples_since_update++;
      
      if (fft_samples_since_update == FFT_SIZE) {
        fft_has_valid_data = true;
      }
      
      // Check if we should perform FFT update
      if (fft_has_valid_data && fft_samples_since_update >= FFT_UPDATE_SAMPLES) {
        // Copy circular buffer to input in correct order and apply precomputed Hamming window
        int start_idx = fft_buffer_idx;  // Oldest sample is at current write position after wrapping
        for (int i = 0; i < FFT_SIZE; ++i) {
          fft_input[i] = fft_circular_buffer[(start_idx + i) % FFT_SIZE] * fft_window[i];
        }
        
        // Perform the FFT
        pffft_transform_ordered(fft_setup, fft_input, fft_output, fft_work, PFFFT_FORWARD);
        
        fft_samples_since_update = 0;
      }
      
      // Accumulate for time-domain display
      time_plot_min_val = std::min(time_plot_min_val, sample);
      time_plot_max_val = std::max(time_plot_max_val, sample);
      time_plot_accumulator++;
      
      if (time_plot_accumulator >= TIME_PLOT_DOWNSAMPLE) {
        time_plot_rolling_window[time_plot_rolling_window_start] = {time_plot_min_val, time_plot_max_val};
        time_plot_rolling_window_start = (time_plot_rolling_window_start + 1) % PLOT_W;
        
        time_plot_min_val = INT32_MAX;
        time_plot_max_val = INT32_MIN;
        time_plot_accumulator = 0;
      }
    }

    // Display based on current screen mode
    if (current_screen == ScreenMode::TIME_DOMAIN) {
      // The FIR filter removes DC, so filtered signal is centered around 0.
      // No need for additional DC subtraction.
      int last_y = 0;

      for (int x = 0; x < PLOT_W; ++x) {
        int window_idx = (time_plot_rolling_window_start + x) % PLOT_W;
        int32_t min_val = time_plot_rolling_window[window_idx].min_val;
        int32_t max_val = time_plot_rolling_window[window_idx].max_val;
        
        // Clamp to range.
        if (min_val < -current_max_val_tnv) {
          min_val = -current_max_val_tnv;
        }
        if (max_val > current_max_val_tnv) {
          max_val = current_max_val_tnv;
        }
        
        int min_y = std::round(min_val * current_time_plot_scale) + TIME_PLOT_MID_SCALE;
        int max_y = std::round(max_val * current_time_plot_scale) + TIME_PLOT_MID_SCALE;
        
        // Draw a vertical line between the min and max values for this bin.
        for (int v_y = min_y; v_y <= max_y; ++v_y) {
          display.drawPixel(x, v_y);
        }
      }
    } else {
      // FREQUENCY_DOMAIN display
      // Display FFT spectrum (continuously updated every 200ms)
      if (fft_has_valid_data) {
        draw_fft_spectrum(fft_output, FFT_SIZE, current_max_val_tnv);
      }
    }

    display.sendBuffer();
  }
}

void loop() {}
