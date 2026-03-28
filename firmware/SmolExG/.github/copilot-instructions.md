# Copilot Instructions for ESP32-C3 DevKitC-02 Project

## Project Overview
This project targets the ESP32-C3 DevKitC-02 board using the Arduino framework, managed via PlatformIO. It interfaces with an ADS1298 ADC chip for high-resolution data acquisition and includes button management for user input.

## Architecture & Key Components
- **src/**: Main source files. 
  - `SmolEMG.cpp`: Entry point, manages device logic, plotting, and hardware setup.
  - `adc.cpp`/`adc.h`: Handles SPI communication and register management for ADS1298.
  - `button_manager.h`: Encapsulates button state logic (short/long press detection).
- **include/**: Header files for reusable components.
- **lib/**: Reserved for custom libraries (currently empty).
- **test/**: PlatformIO test runner setup (see `README`).

## Build & Workflow
- **Build/Upload**: Use PlatformIO commands (`pio run`, `pio upload`).
- **Monitor**: Serial monitor speed is set to 460800 baud (see `platformio.ini`).
- **Unit Testing**: PlatformIO's test runner is supported. See [PlatformIO Unit Testing docs](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html).

## Project-Specific Patterns
- **Pin Assignments**: All hardware pin mappings are defined as `constexpr` in `SmolEMG.cpp` and `adc.h`.
- **Data Rate**: ADC sampling rate is controlled via `DATA_RATE_SETTING` and `DATA_RATE_SPS`.
- **Button Handling**: Use `ButtonManager` class for debounced, stateful button input.
- **SPI Communication**: All SPI operations for ADS1298 are wrapped in functions (`read_register`, `write_register`, `send_command`).
- **Plotting**: Display logic is managed in `SmolEMG.cpp` using U8g2lib for OLED screens.

## External Dependencies
- **Arduino Framework**: All code assumes Arduino APIs.
- **U8g2lib**: For display handling.
- **PlatformIO**: Project configuration and build system.
- **lib_extra_dirs**: Custom libraries can be added via `platformio.ini`.

## Integration & Communication
- **ADC**: SPI-based communication with ADS1298, with timing constraints documented in `adc.h`.
- **Buttons**: Active-low logic, state transitions managed by `ButtonManager`.

## Example Patterns
- **ButtonManager Usage**:
  ```cpp
  ButtonManager btn(BUTTON_1_PIN);
  auto state = btn.Update();
  if (state == ButtonManager::State::SHORT) { /* handle short press */ }
  ```
- **SPI Register Read**:
  ```cpp
  uint8_t id = read_register(spi, ADS1298_REG_ID);
  ```

## Conventions
- Prefer `constexpr` for hardware constants.
- Use PlatformIO for all build, upload, and test operations.
- Place reusable code in `include/`, main logic in `src/`.

## References
- [platformio.ini](platformio.ini): Build config, board, framework, monitor speed, library paths.
- [test/README](test/README): Unit testing setup.
- [include/adc.h](include/adc.h): ADC register/timing definitions.
- [src/SmolEMG.cpp](src/SmolEMG.cpp): Main application logic.

---
_Review and update these instructions as the project evolves. If any section is unclear or incomplete, please provide feedback for improvement._
