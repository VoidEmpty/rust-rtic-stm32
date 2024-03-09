## Сборка

Самый простой способ установки раста на любую операционную систему - через [rustup](https://rustup.rs).

Далее необходимо установить соответствующий таргет для сборки под Cortex-M4F:

```shell
rustup target add thumbv7em-none-eabihf
```

Для отладки потребуется [probe-rs](https://probe.rs). И проверка переполнения стека [flip-link](https://github.com/knurling-rs/flip-link).

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

## Поддерживаемые команды

Протокол в формате JSON. Каждое сообщение завершается нулевым байтом.

### Включение регулярной отправки статистики

Можно отправлять повторно для изменения частоты отправки статистики.

```json
{
    "cmd": 1,
    "delay_time": 1000   // в милисекундах
}
```

### Отключение отправки статистики

```json
{
    "cmd": 0
}
```

### Поворот шагового мотора на заданный угол

```json
{
    "cmd": 2,
    "angle": 32.3  // в градусах
}
```

### Пример отправляемых данных

```json
{
    "gps_data": {
        "latitude": 5546.959,
        "lat_dir": "North",
        "longitude": 3740.692,
        "lon_dir": "East",
        "time": 123035.0
    },
    "quat": {
        "q0": 1,
        "q1": 2,
        "q2": 3,
        "q3": 4
    },
    "motor_data": {
        "motor_state": "Stopped",
        "motor_angle": 60.0
    }
}
```
