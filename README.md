# RP2040-FreeRTOS BNO055 Sensor Application

This repo contains my base project for a [FreeRTOS](https://freertos.org/) application to be ran on the
[Raspberry Pi RP2040 microcontroller](https://www.raspberrypi.com/products/rp2040/).
This was developed using a [Adafruit Feather RP2040](https://www.adafruit.com/product/4884).

The application connects using the default I2C interface to a BNO055 sensor and extracts pitch, yaw, and roll from it and prints it to stdio.
Stdio is configured to be over UART using interface UART0 or USB.

Based on the work done by smittytone, details [in this blog post](https://blog.smittytone.net/2022/02/24/how-to-use-freertos-with-the-raspberry-pi-pico/).


## Project Structure

```
/RP2040-FreeRTOS
|
|___/app                    // Application source code
|   |___CMakeLists.txt      // Application-level CMake config file
|
|___/config
|   |___FreeRTOSConfig.h    // FreeRTOS project config file
|
|___/FreeRTOS-Kernel        // FreeRTOS kernel files, included as a submodule
|___/pico-sdk               // Raspberry Pi Pico SDK, included as a submodule
|
|___CMakeLists.txt          // Top-level project CMake config file
|___pico_sdk_import.cmake   // Raspberry Pi Pico SDK CMake import script
|___deploy.sh               // Build-and-deploy shell script
|
|___rp2040.code-workspace   // Visual Studio Code workspace
|___rp2040.xcworkspace      // Xcode workspace
|
|___README.md
|___LICENSE.md
```

## Prerequisites

To use the code in this repo, your system must be set up for RP2040 C/C++ development.
See [this blog post of smittyone](https://blog.smittytone.net/2021/02/02/program-raspberry-pi-pico-c-mac/) for setup details.


## Usage

1. Clone the repo.
2. Enter the repo.
3. Install the submodules: `git submodule update --init --recursive`.
4. Create the build directory: `mkdir build`.
5. Manually build the app: `cmake --build build`.
6. Connect your device so it’s ready for file transfer.
7. Press the BOOTSEL button followed by the RESET button to put the board in bootloading mode. The board should appear as a USB drive.
8. Copy the file `${PROJECT ROOT}/build/app/rpi-pico-imu.uf2` to the USB device mounted by the bootloading process.
9. The board should restart and run the application.


## Raspberry Pi RP2040 Variants

If your board is a variant of the Raspberry Pi Pico, you need to specify this using the `PICO_BOARD` cmake flag in step 5 of usage.
For example, the Adafruit Feather RP2040:

```shell
cmake --build build -DPICO_BOARD=adafruit_feather_rp2040
```


## Debug vs Release

You can switch between build types when you make the `cmake` call in step 5 of usage. A debug build is made explicit with:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
```

For a release build, which among various optimisations omits UART debugging code, call:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
```

Follow both of these commands with the usual

```shell
cmake --build build
```


## IDEs

Workspace files are included for the Visual Studio Code and Xcode IDEs.


## Credits

This work was inspired by work done on [Twilio Microvisor FreeRTOS Demo code](https://github.com/twilio/twilio-microvisor-freertos), but the version of the `FreeRTOSConfig.h` file included here was derived from [work by @yunka2](https://github.com/yunkya2/pico-freertos-sample).


## Copyright and Licences

Application template gerated from source code © 2023, by Tony Smith and licensed under the terms of the [MIT Licence](./LICENSE.md).

[FreeRTOS](https://freertos.org/) © 2021, Amazon Web Services, Inc. It is also licensed under the terms of the [MIT Licence](./LICENSE.md).

The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) is © 2020, Raspberry Pi (Trading) Ltd. It is licensed under the terms of the [BSD 3-Clause "New" or "Revised" Licence](https://github.com/raspberrypi/pico-sdk/blob/master/LICENSE.TXT).
