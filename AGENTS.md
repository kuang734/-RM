# Project Instructions

## Scope

This repository currently contains the STM32F407 firmware project. ESP32 code may be added as a sibling ESP-IDF or PlatformIO project; keep its build files in that project directory and do not mix ESP32 sources into the STM32 target.

## STM32F407 build and verification

- Preferred configure/build: `cmake --preset Debug` followed by `cmake --build --preset Debug`.
- Makefile fallback: `make -j` (on Windows, `mingw32-make -j`).
- The ARM GNU Embedded Toolchain must provide `arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, and `arm-none-eabi-size` on `PATH`.
- CMake exports `build/Debug/compile_commands.json`; use it for clangd and code navigation.
- Do not change the MCU, linker script, floating-point ABI, or HAL defines without checking the matching startup file and linker script.

## ESP32 projects

- For ESP-IDF projects, use the ESP-IDF environment and run `idf.py build`; the project directory must contain `CMakeLists.txt` and `sdkconfig` or `sdkconfig.defaults`.
- For PlatformIO projects, run `pio run`; the project directory must contain `platformio.ini`.
- Prefer the project's generated `compile_commands.json` for clangd. Never hard-code a user-specific ESP-IDF or Python installation path; use `IDF_PATH`, `IDF_TOOLS_PATH`, or the editor's environment instead.

## Editing and tests

- Keep hardware-specific changes scoped to the relevant project.
- Run the narrowest available build after code changes and report missing toolchains explicitly.
