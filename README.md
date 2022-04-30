# rustykeys-mouse

## requirement

for cargo run\
`cargo install elf2uf2-rs`

- you may also want to use probe-run as runner\
`cargo install probe-run`

for cargo embed\
`cargo install cargo-embed`

## usage

to build\
`cargo build --release`

to program using bootloader\
`cargo run --release`

- edit [.cargo/config.toml](./.cargo/config.toml) to use probe-run as runner.

to program using debugger supported by probe-rs\
`cargo embed --release`
