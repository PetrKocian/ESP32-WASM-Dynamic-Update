# ESP32 Wasm Demo
This folder contains an ESP32 project, that tries to listens on UART for incoming Wasm binaries and then tries to execute them. Each new Wasm binary overwrites the current running one.


## Build and run

The project is built and run using [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) and [WAMR](https://github.com/bytecodealliance/wasm-micro-runtime/tree/main).

Before running the script, ensure ESP-IDF is properly set up and edit the `WAMR_PATH` in `build_and_run.sh` to match the directory of WAMR.

To build the project the first time, use the command `build_and_run.sh`.

Afterwards, `idf.py build` and `idf.py flash monitor` can be used.
