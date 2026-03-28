#include <Arduino.h>

#include <driver/adc.h>

#include "adc.h"

// Static SPI instance for ADS1298 communication.
static SPIClass adc_spi(FSPI);

uint8_t read_register(SPIClass& spi, uint8_t reg_addr) {
  spi.beginTransaction(SPISettings(ADS1298_SPI_CLK_RATE, MSBFIRST, SPI_MODE1));
  digitalWrite(spi.pinSS(), LOW);
  uint8_t opcode0 = 0x20 | reg_addr;
  uint8_t opcode1 = 0x00; // read one register.
  spi.transfer(opcode0);
  spi.transfer(opcode1);
  uint8_t ret = spi.transfer(0x00);
  delayMicroseconds(ADS1298_SPI_CS_MIN_DELAY_US);
  digitalWrite(spi.pinSS(), HIGH);
  delayMicroseconds(ADS1298_SPI_CS_MIN_PULSE_US);
  spi.endTransaction();
  return ret;
}

void write_register(SPIClass& spi, uint8_t reg_addr, uint8_t value) {
  spi.beginTransaction(SPISettings(ADS1298_SPI_CLK_RATE, MSBFIRST, SPI_MODE1));
  digitalWrite(spi.pinSS(), LOW);
  uint8_t opcode0 = 0x40 | reg_addr;
  uint8_t opcode1 = 0x00; // write one register.
  spi.transfer(opcode0);
  spi.transfer(opcode1);
  spi.transfer(value);
  delayMicroseconds(ADS1298_SPI_CS_MIN_DELAY_US);
  digitalWrite(spi.pinSS(), HIGH);
  delayMicroseconds(ADS1298_SPI_CS_MIN_PULSE_US);
  spi.endTransaction();
}

void send_command(SPIClass& spi, uint8_t opcode) {
  spi.beginTransaction(SPISettings(ADS1298_SPI_CLK_RATE, MSBFIRST, SPI_MODE1));
  digitalWrite(spi.pinSS(), LOW);
  spi.transfer(opcode);
  delayMicroseconds(ADS1298_SPI_CS_MIN_DELAY_US);
  digitalWrite(spi.pinSS(), HIGH);
  delayMicroseconds(ADS1298_SPI_CS_MIN_PULSE_US);
  spi.endTransaction();
}

int32_t IRAM_ATTR ads1298_code_to_tenths_nanovolts(int32_t raw_code, int gain) {
  // Convert raw ADC code to signed tenths of nanovolts (10nV units, integer-only, ISR-safe)
  // Formula: voltage_tnV = raw_code * V_REF_uV * 100 / (ADC_MAX_CODE * gain)
  int64_t numerator = (int64_t)raw_code * ADS1298_VREF_MICROVOLTS * 100LL;
  int64_t divisor = (int64_t)ADS1298_ADC_MAX_CODE * gain;
  int32_t tnv = (int32_t)(numerator / divisor);
  return tnv;
}

void read_next_samples(SPIClass& spi, int32_t *out, uint8_t bytes_per_channel) {
  // Wait for data ready.
  while (digitalRead(ADS1298_DRDY_PIN) == HIGH) {}

  spi.beginTransaction(SPISettings(ADS1298_SPI_CLK_RATE, MSBFIRST, SPI_MODE1));
  digitalWrite(spi.pinSS(), LOW);

  // First get the status word.
  uint8_t buf[3] = {0};
  spi.transfer(buf, 3);

  for (int ch = 0; ch < 8; ++ch) {
    buf[0] = buf[1] = buf[2] = 0;
    spi.transfer(buf, bytes_per_channel);
    uint32_t out_raw = 0;
    if (bytes_per_channel == 2) {
      out_raw |= buf[0];
      out_raw <<= 8;
      out_raw |= buf[1];
      if (buf[0] & 0x80) {
        // Sign-extend.
        out_raw |= 0xffff0000;
      }
    } else if (bytes_per_channel == 3) {
      out_raw |= buf[0];
      out_raw <<= 8;
      out_raw |= buf[1];
      out_raw <<= 8;
      out_raw |= buf[2];
      if (buf[0] & 0x80) {
        // Sign-extend.
        out_raw |= 0xff000000;
      }
    }
    memcpy(&out[ch], &out_raw, 4);
  }

  delayMicroseconds(ADS1298_SPI_CS_MIN_DELAY_US);
  digitalWrite(spi.pinSS(), HIGH);
  delayMicroseconds(ADS1298_SPI_CS_MIN_PULSE_US);
  spi.endTransaction();
}

// This is the ring buffer for async ADC samples.
// We only support one channel currently. Output is normalized to the
// reference gain (12x), making it gain-agnostic regardless of channel gain.
volatile int32_t adc_buffer[RING_BUFFER_SIZE];
volatile int adc_buffer_head = 0; // Written by ISR
volatile int adc_buffer_tail = 0; // Read by Main Loop
volatile int adc_spi_ss_pin = 0;
SPIClass* volatile adc_spi_for_isr = nullptr;

std::size_t adc_data_available() {
    noInterrupts();
    int count = (adc_buffer_head - adc_buffer_tail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
    interrupts();
    return static_cast<std::size_t>(count);
}

int32_t read_adc_buffer() {
  int32_t ret;
  noInterrupts();
  if (adc_buffer_head != adc_buffer_tail) {
    ret = adc_buffer[adc_buffer_tail];
    adc_buffer_tail = (adc_buffer_tail + 1) % RING_BUFFER_SIZE;
  }
  interrupts();
  return ret;
}

void IRAM_ATTR ads1298_isr() {
  // Calculate the next position for the head
  int next_head = (adc_buffer_head + 1) % RING_BUFFER_SIZE;

  // OVERWRITE LOGIC:
  // If the next head position is where the tail is, the buffer is full.
  // We "push" the tail forward to make room, discarding the oldest sample.
  if (next_head == adc_buffer_tail) {
      adc_buffer_tail = (adc_buffer_tail + 1) % RING_BUFFER_SIZE;
  }

  adc_spi_for_isr->beginTransaction(SPISettings(ADS1298_SPI_CLK_RATE, MSBFIRST, SPI_MODE1));
  digitalWrite(adc_spi_ss_pin, LOW);

  // Read and discard status word (3 bytes)
  uint8_t s1 = adc_spi_for_isr->transfer(0);
  uint8_t s2 = adc_spi_for_isr->transfer(0);
  uint8_t s3 = adc_spi_for_isr->transfer(0);

  // Read 8 Channels (24-bit mode)
  for (int i = 0; i < 8; i++) {
      uint8_t b1 = adc_spi_for_isr->transfer(0);
      uint8_t b2 = adc_spi_for_isr->transfer(0);
      uint8_t b3 = adc_spi_for_isr->transfer(0);
      
      if (i == ADC_CHANNEL) {
        // Combine and sign-extend using arithmetic right shift
        int32_t raw_code = ((int32_t)b1 << 24) | ((int32_t)b2 << 16) | ((int32_t)b3 << 8);
        raw_code = raw_code >> 8;
        
        // Convert to signed tenths of nanovolts with actual gain
        int32_t val_tnv = ads1298_code_to_tenths_nanovolts(raw_code, ADS1298_CH0_GAIN);
        adc_buffer[adc_buffer_head] = val_tnv;
      }
  }

  digitalWrite(adc_spi_ss_pin, HIGH);
  adc_spi_for_isr->endTransaction();

  // Update the head to the next position
  adc_buffer_head = next_head;
}

bool ads1298_init(
  uint8_t data_rate_setting,
  int cs_pin,
  int do_pin,
  int din_pin,
  int clk_pin,
  int drdy_pin
) {
  // Set up ADC SPI interface.
  pinMode(do_pin, OUTPUT);
  pinMode(din_pin, INPUT);
  pinMode(clk_pin, OUTPUT);

  adc_spi.begin(clk_pin, din_pin, do_pin, cs_pin);
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  // Reset the chip, and wait 18 t_clk cycles (8.7uS).
  send_command(adc_spi, ADS1298_OPCODE_RESET);
  delayMicroseconds(9);

  // The chip boots in continuous read mode which ignores register reads.
  // Let's stop that.
  send_command(adc_spi, ADS1298_OPCODE_SDATAC);

  // Do a sanity check (read chip ID).
  uint8_t adc_chip_id = read_register(adc_spi, ADS1298_REG_ID);

  if (adc_chip_id != 0x92) {
    return false;
  }

  // Set up internal reference (turn on reference buffer).
  write_register(adc_spi, ADS1298_REG_CONFIG3, 0xc0);

  // High resolution mode with the correct data rate.
  write_register(adc_spi, ADS1298_REG_CONFIG1, 0x80 | data_rate_setting);

  // Write 0 to the reserved bits in CONFIG2. Why? Don't know. Datasheet
  // says so. Also enable the internal test signal.
  write_register(adc_spi, ADS1298_REG_CONFIG2, 0x10);

  // Set all channels to power down with input shorted except channel 0 to test.
  for (int ch = 0; ch < 8; ++ch) {
    uint8_t reg_val = 0x81;
    if (ch == 0) {
      if (ADC_TEST_MODE) {
        reg_val = 0x05;
      } else {
        reg_val = 0x00;
      }
    }
    write_register(adc_spi, ADS1298_REG_CHnSET_BASE + ch, reg_val);
  }

  // Set up interrupt to read into ring buffer.
  adc_spi_ss_pin = cs_pin;
  adc_spi_for_isr = &adc_spi;
  attachInterrupt(digitalPinToInterrupt(drdy_pin), ads1298_isr, FALLING);

  // Start conversion and start continuous read.
  send_command(adc_spi, ADS1298_OPCODE_START);
  send_command(adc_spi, ADS1298_OPCODE_RDATAC);

  return true;
}