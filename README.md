# servo-reg-dev-board
Rust code for a Raspberry Pi Pico 2W that controls a custom electronic gas regulator

# Building

## Pico Program

cargo run --release -p pico_program

## Generating Python bindings

cargo run --release -p bindings_gen --target x86_64-pc-windows-msvc