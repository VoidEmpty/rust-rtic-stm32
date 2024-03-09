## Сборка

Самый простой способ установки раста на любую операционную систему - через [rustup](https://rustup.rs).

Далее необходимо установить соответствующий таргет для сборки под Cortex-M4F:

```shell
rustup target add thumbv7em-none-eabihf
```

Для отладки потребуется [probe-run](https://github.com/knurling-rs/probe-run). И проверка переполнения стека [flip-link](https://github.com/knurling-rs/flip-link).

```shell
cargo install probe-rs --features cli
cargo install flip-link
```

Для прошивки достаточно выполнить команду:
```shell
cargo embed --release
```

Для запуска с логированием в консоль:
```shell
cargo run --release
```

Для запуска тестов:
```shell
cargo test
```

Установить драйвер [ST-LINK](https://www.st.com/en/development-tools/stsw-link009.html) для прошивки и отладки.

Для удобства разработки рекомендуется использовать [Visual Studio Code](https://code.visualstudio.com) c следующим набором расширений:

```shell
code --install-extension \
rust-lang.rust-analyzer \
probe-rs.probe-rs-debugger \
tamasfe.even-better-toml \
serayuzgur.crates
```