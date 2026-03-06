# Build prerequisites

## Ubuntu / Debian

```sh
# ARM cross-compiler + binutils
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# C standard library headers and runtime for bare-metal ARM (provides string.h etc.)
sudo apt install libnewlib-arm-none-eabi

# Rust target
rustup target add thumbv7em-none-eabihf
```

## Building the example

```sh
cd example-sfy
cargo build --release
```

## Flashing and RTT output

```sh
# With probe-rs (recommended)
probe-rs run --chip AMA3B1KK-KBR target/thumbv7em-none-eabihf/release/example-sfy

# Or with the SparkFun SVL bootloader over USB serial
cargo run --release
```
