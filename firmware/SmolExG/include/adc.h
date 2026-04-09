#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <SPI.h>

#include <chrono>
#include <cstdint>

#include <driver/adc.h>

constexpr int ADS1298_DRDY_PIN = 7;

constexpr float ADS1298_VREF = 2.4f;

constexpr std::size_t RING_BUFFER_SIZE = 4096;

constexpr uint8_t ADC_CHANNEL = 0; // We only read the first channel right now.

constexpr bool ADC_TEST_MODE = false;

// PGA Gain settings (CHnSET register bits 6:4)
constexpr uint8_t ADS1298_GAIN_CODE_6X  = 0b000;
constexpr uint8_t ADS1298_GAIN_CODE_1X  = 0b001;
constexpr uint8_t ADS1298_GAIN_CODE_2X  = 0b010;
constexpr uint8_t ADS1298_GAIN_CODE_3X  = 0b011;
constexpr uint8_t ADS1298_GAIN_CODE_4X  = 0b100;
constexpr uint8_t ADS1298_GAIN_CODE_8X  = 0b101;
constexpr uint8_t ADS1298_GAIN_CODE_12X = 0b110;

// Channel 0 gain setting (bits 6:4 of 0x00 = 000 = 6x gain)
constexpr uint8_t ADS1298_CH0_GAIN_SETTING = ADS1298_GAIN_CODE_6X;

// Gain lookup table indexed by gain code
constexpr int ADS1298_GAIN_TABLE[] = {6, 1, 2, 3, 4, 8, 12};

// Raw ADC output range (24-bit signed)
constexpr int32_t ADS1298_RAW_ADC_MIN = -8388608;  // -2^23
constexpr int32_t ADS1298_RAW_ADC_MAX = 8388607;   // 2^23 - 1

// Reference gain for normalization (highest available gain)
constexpr int ADS1298_REFERENCE_GAIN = 12;

constexpr int ADS1298_CH0_GAIN = ADS1298_GAIN_TABLE[ADS1298_CH0_GAIN_SETTING];

// Integer constants for microvolts conversion (V_REF in microvolts)
constexpr int32_t ADS1298_VREF_MICROVOLTS = 2400000;  // 2.4V in microvolts
constexpr int32_t ADS1298_ADC_MAX_CODE = 8388607;     // 2^23 - 1

// Get the min/max ADC values scaled to reference gain (12x) for a given gain code
constexpr int32_t ads1298_get_scaled_min(uint8_t gain_code) {
  return (int64_t)ADS1298_RAW_ADC_MIN * ADS1298_REFERENCE_GAIN / ADS1298_GAIN_TABLE[gain_code];
}

constexpr int32_t ads1298_get_scaled_max(uint8_t gain_code) {
  return (int64_t)ADS1298_RAW_ADC_MAX * ADS1298_REFERENCE_GAIN / ADS1298_GAIN_TABLE[gain_code];
}

// The chip supports much higher data rate, but the ADS1298 has many operations that require a minimum delay
// of 4 t_clk (4 internal clock cycles = 1.96uS). If our byte transfer time is longer than that, we don't
// need to worry about it, and this makes our life much easier. At 2MHz, 8 bits is 4uS. This is more than
// fast enough for us.
constexpr int ADS1298_SPI_CLK_RATE = 2000000;
constexpr int ADS1298_SPI_CS_MIN_PULSE_US = 1;  // 2 t_clk cycles.
constexpr int ADS1298_SPI_CS_MIN_DELAY_US = 2;  // 4 t_clk cycles.

constexpr uint8_t ADS1298_REG_ID          = 0x00; // Device ID Register
constexpr uint8_t ADS1298_REG_CONFIG1     = 0x01; // Configuration Register 1
constexpr uint8_t ADS1298_REG_CONFIG2     = 0x02; // Configuration Register 2
constexpr uint8_t ADS1298_REG_CONFIG3     = 0x03; // Configuration Register 3
constexpr uint8_t ADS1298_REG_LOFF        = 0x04; // Lead-Off Control Register
constexpr uint8_t ADS1298_REG_CHnSET_BASE      = 0x05; // Channel 1 Settings
constexpr uint8_t ADS1298_REG_RLD_SENSP   = 0x0D; // RLD Positive Signal Derivation
constexpr uint8_t ADS1298_REG_RLD_SENSN   = 0x0E; // RLD Negative Signal Derivation
constexpr uint8_t ADS1298_REG_LOFF_SENSP  = 0x0F; // Positive Signal Lead-Off Detection
constexpr uint8_t ADS1298_REG_LOFF_SENSN  = 0x10; // Negative Signal Lead-Off Detection
constexpr uint8_t ADS1298_REG_LOFF_FLIP   = 0x11; // Lead-Off Flip Register
constexpr uint8_t ADS1298_REG_LOFF_STATP  = 0x12; // Lead-Off Positive Signal Status
constexpr uint8_t ADS1298_REG_LOFF_STATN  = 0x13; // Lead-Off Negative Signal Status
constexpr uint8_t ADS1298_REG_GPIO        = 0x14; // General-Purpose I/O Register
constexpr uint8_t ADS1298_REG_PACE        = 0x15; // Pace Detect Register
constexpr uint8_t ADS1298_REG_RESP        = 0x16; // Respiration Control Register
constexpr uint8_t ADS1298_REG_CONFIG4     = 0x17; // Configuration Register 4
constexpr uint8_t ADS1298_REG_WCT1        = 0x18; // WCT and Augmented Lead Control
constexpr uint8_t ADS1298_REG_WCT2        = 0x19; // WCT Control Register

constexpr uint8_t ADS1298_OPCODE_WAKEUP = 0x02;
constexpr uint8_t ADS1298_OPCODE_STANDBY = 0x04;
constexpr uint8_t ADS1298_OPCODE_RESET = 0x06;
constexpr uint8_t ADS1298_OPCODE_START = 0x08;
constexpr uint8_t ADS1298_OPCODE_STOP = 0x10;

constexpr uint8_t ADS1298_OPCODE_RDATAC = 0x10;
constexpr uint8_t ADS1298_OPCODE_SDATAC = 0x11;
constexpr uint8_t ADS1298_OPCODE_RDATA = 0x12;

extern volatile int adc_spi_ss_pin;
extern SPIClass* volatile adc_spi_for_isr;

// Convert raw ADC code to signed microvolts
// Input: raw_code (24-bit two's complement ADC value), gain (PGA gain)
// Output: signed voltage in microvolts
int32_t ads1298_code_to_microvolts(int32_t raw_code, int gain);

uint8_t read_register(SPIClass& spi, uint8_t reg_addr);

void write_register(SPIClass& spi, uint8_t reg_addr, uint8_t value);

void send_command(SPIClass& spi, uint8_t opcode);

void read_next_samples(SPIClass& spi, int32_t *out, uint8_t bytes_per_channel);

std::size_t adc_data_available();

int32_t read_adc_buffer();

void IRAM_ATTR ads1298_isr();

bool ads1298_init(
  uint8_t data_rate_setting,
  int cs_pin,
  int do_pin,
  int din_pin,
  int clk_pin,
  int drdy_pin
);

#endif // ADC_H
