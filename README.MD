# x.copter

`xcopter` - асинхронная Python-библиотека для управления коптером через MAVLink.
Основной контракт описан абстрактным классом `XCopter` в
`src/xcopter/xcopter.py`, а реализация для ArduCopter находится в
`XArduCopter`.

Все методы интерфейса `XCopter` являются `async`, поэтому вызываются через
`await`. В параметрах движения используются метрические единицы:

- расстояние и высота: метры;
- скорость: метры в секунду;
- углы: градусы;
- частота: герцы;
- время ожидания: секунды.

## Установка

```bash
pip install .
```

Внешние MAVLink-зависимости не требуются: разбор и сборка используемых
MAVLink-сообщений реализованы внутри пакета на стандартной библиотеке Python.

## Пример

```python
import asyncio
from xcopter import XArduCopter


async def main():
    copter = XArduCopter()

    try:
        await copter.connect("udp:127.0.0.1:14550", baud=115200, gcs=True)
        await copter.guide()
        await copter.arm()
        await copter.takeoff(3, speed=0.5)
        await copter.rotate(90, 30)
        await copter.shift(forward=5, up=1, speed=1)
        await copter.rtl()
    finally:
        await copter.close()


asyncio.run(main())
```

## Интерфейс `XCopter`

Ниже перечислены все методы, объявленные в `src/xcopter/xcopter.py`.
Реализация обычно возвращает `True` при успешной отправке или завершении
команды, либо значение датчика/параметра. При ошибках реализации выбрасывают
`CopterError`.

### `armed(timeout=5)`

Проверяет, находится ли коптер в armed-состоянии.

- `timeout` - время ожидания MAVLink-сообщения `HEARTBEAT`.
- Возвращает `True`, если коптер armed, иначе `False`.

Пример:

```python
is_armed = await copter.armed()
```

### `arm(timeout=5)`

Отправляет команду постановки на вооружение.

- `timeout` - время ожидания `COMMAND_ACK`.
- Возвращает `True`, если команда принята автопилотом.
- Если коптер уже armed, реализация возвращает `True` без повторной команды.

Пример:

```python
await copter.arm()
```

### `disarm(timeout=5, force=True)`

Отправляет команду снятия с вооружения.

- `timeout` - время ожидания `COMMAND_ACK`.
- `force` - если `True`, используется force-disarm код ArduPilot `21196`.
- Возвращает `True`, если команда принята автопилотом.

Пример:

```python
await copter.disarm()
await copter.disarm(force=False)
```

### `mode(timeout=5)`

Читает текущий режим полета.

- `timeout` - время ожидания `HEARTBEAT`.
- Возвращает строку с названием режима, например `GUIDED`, `STABILIZE`, `RTL`.

Пример:

```python
mode = await copter.mode()
```

### `land(descend=2, land=0.5)`

Переводит коптер в режим посадки.

- `descend` - скорость снижения до посадочной фазы, м/с.
- `land` - финальная скорость посадки, м/с.
- Возвращает `True` после настройки параметров и отправки режима `LAND`.

Примечание: в `XCopter` объявлен дополнительный параметр `timeout=5`, но
текущая реализация `Land.land()` его не принимает.

Пример:

```python
await copter.land(descend=2, land=0.5)
```

### `rtl(speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5)`

Включает режим Return To Launch и ожидает завершения посадки.

- `speed` - горизонтальная скорость возврата, м/с.
- `climb` - скорость набора высоты, м/с.
- `descend` - скорость снижения, м/с.
- `land` - финальная скорость посадки, м/с.
- `rtl_alt` - высота возврата RTL, м.
- `vert_accel` - вертикальное ускорение, м/с².
- `timeout` - время ожидания heartbeat при проверке armed-состояния.
- Возвращает `True`, когда коптер перестал быть armed.

Пример:

```python
await copter.rtl(speed=5, rtl_alt=20)
```

### `smart_rtl(speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5)`

Включает режим Smart RTL и ожидает завершения посадки.

Параметры такие же, как у `rtl()`. Возвращает `True`, когда коптер перестал
быть armed.

Пример:

```python
await copter.smart_rtl(speed=5, rtl_alt=20)
```

### `guide(timeout=5)`

Переводит коптер в режим `GUIDED`.

- `timeout` - время ожидания подтверждения смены режима.
- Возвращает `True`, если смена режима принята автопилотом.

Реализация отправляет `COMMAND_LONG` с `MAV_CMD_DO_SET_MODE` и ожидает
`COMMAND_ACK`.

Пример:

```python
await copter.guide()
```

### `guided(timeout=5)`

Проверяет, активен ли режим `GUIDED`.

- `timeout` - время ожидания данных для определения текущего режима.
- Возвращает `True`, если текущий режим `GUIDED`.
- Если режим отличается, реализация выбрасывает `CopterError`.

Пример:

```python
is_guided = await copter.guided()
```

### `takeoff(target_height, epsilon=0.3, speed=3, hold_time=3, vert_accel=2, takeoff_timeout=300, timeout=1)`

Запускает взлет до заданной относительной высоты.

- `target_height` - целевая высота над точкой взлета, м.
- `epsilon` - допустимое отклонение от целевой высоты, м.
- `speed` - скорость взлета, м/с.
- `hold_time` - сколько секунд высота должна удерживаться в пределах `epsilon`.
- `vert_accel` - вертикальное ускорение, м/с².
- `takeoff_timeout` - общий таймаут достижения высоты.
- `timeout` - время ожидания `GLOBAL_POSITION_INT`.
- Возвращает `True` после достижения и удержания целевой высоты.

Пример:

```python
await copter.takeoff(3, speed=0.5)
```

### `altitude(timeout=5)`

Читает абсолютную высоту.

- `timeout` - время ожидания `VFR_HUD`.
- Возвращает `alt` из `VFR_HUD` в метрах.

Пример:

```python
altitude = await copter.altitude()
```

### `heading(timeout=5)`

Читает текущий курс коптера.

- `timeout` - время ожидания `ATTITUDE`.
- Возвращает yaw, преобразованный из радиан в градусы и нормализованный в диапазон `0..360`.

Пример:

```python
heading = await copter.heading()
```

### `height(timeout=1)`

Читает относительную высоту над точкой взлета.

- `timeout` - время ожидания `GLOBAL_POSITION_INT`.
- Возвращает `relative_alt` в метрах.

Пример:

```python
height = await copter.height()
```

### `beep(tone=2000, duration=1, count=1)`

Отправляет команду на пищалку.

- `tone` - частота сигнала, Гц.
- `duration` - длительность одного сигнала, с.
- `count` - количество повторов.
- Возвращает `True` после отправки команды.

Пример:

```python
await copter.beep(tone=2000, duration=1, count=2)
```

### `location(timeout=1)`

Читает локальную позицию в системе `LOCAL_POSITION_NED`.

- `timeout` - время ожидания `LOCAL_POSITION_NED`.
- Возвращает словарь `{"x": float, "y": float}` в метрах.

Пример:

```python
location = await copter.location()
```

### `shift(forward=0.0, right=0.0, up=0.0, speed=0.1, epsilon=0.05, shift_timeout=300, timeout=5)`

Смещает коптер относительно его текущей ориентации.

- `forward` - смещение вперед, м.
- `right` - смещение вправо, м.
- `up` - смещение вверх, м.
- `speed` - скорость перемещения, м/с.
- `epsilon` - допустимое отклонение по пройденной дистанции, м.
- `shift_timeout` - общий таймаут перемещения.
- `timeout` - время ожидания `LOCAL_POSITION_NED` при контроле позиции.
- Возвращает `True`, когда целевая дистанция достигнута.

Пример:

```python
await copter.shift(forward=5, right=0, up=1, speed=1)
```

### `rotate(angle, rate, tolerance=3, rotate_timeout=100, timeout=5)`

Поворачивает коптер на относительный угол.

- `angle` - угол поворота, градусы. Положительное значение - по часовой стрелке, отрицательное - против.
- `rate` - скорость поворота, градусы/с.
- `tolerance` - допустимая ошибка по yaw, градусы.
- `rotate_timeout` - общий таймаут поворота.
- `timeout` - время ожидания `ATTITUDE`.
- Возвращает `True`, когда ошибка по углу стала меньше или равна `tolerance`.

Примечание: в `XCopter` параметр называется `rotate_timeout`, но в текущей
реализации `Rotate.rotate()` есть опечатка `rorate_timeout`. Позиционные
аргументы работают, но именованный аргумент `rotate_timeout=` потребует
исправления реализации.

Пример:

```python
await copter.rotate(90, 30)
```

### `position(timeout=5)`

Читает глобальную позицию.

- `timeout` - время ожидания `GLOBAL_POSITION_INT`.
- Возвращает словарь `{"lat": float, "lon": float, "alt": float}`.
- `lat` и `lon` возвращаются в градусах, `alt` - абсолютная высота AMSL в метрах.

Пример:

```python
position = await copter.position()
```

### `navigate(lat, lon, alt, speed=5, epsilon=0.5, hz=10, navigation_timeout=600, timeout=60)`

Ведет коптер к глобальной координате.

- `lat` - целевая широта в градусах.
- `lon` - целевая долгота в градусах.
- `alt` - целевая абсолютная высота AMSL, м.
- `speed` - горизонтальная скорость, м/с.
- `epsilon` - допустимое отклонение по горизонтали и вертикали, м.
- `hz` - частота отправки команды позиции.
- `navigation_timeout` - общий таймаут навигации.
- `timeout` - время ожидания текущей позиции.
- Возвращает `True`, когда позиция достигнута в пределах `epsilon`.

Пример:

```python
position = await copter.position()

await copter.navigate(
    56.95688850666043,
    24.250718748307154,
    position["alt"],
    speed=3,
    epsilon=0.5,
)
```

### `stop()`

Останавливает движение коптера командой нулевой скорости.

- Возвращает `True` после отправки команды остановки.

Примечание: в `XCopter` объявлен параметр `timeout=5`, но текущая реализация
`Local.stop()` его не принимает.

Пример:

```python
await copter.stop()
```

### `close()`

Закрывает соединение с коптером.

В `XArduCopter.close()` также отменяется фоновая отправка heartbeat, если
соединение было открыто с `gcs=True`.

Пример:

```python
await copter.close()
```

### `voltage()`

Читает напряжение батареи.

- Текущая реализация также принимает необязательный `timeout=5`.
- Возвращает напряжение в вольтах из `SYS_STATUS.voltage_battery`.

Пример:

```python
voltage = await copter.voltage()
```

### `current()`

Читает ток батареи.

- Текущая реализация также принимает необязательный `timeout=5`.
- Возвращает ток в амперах из `SYS_STATUS.current_battery`.

Пример:

```python
current = await copter.current()
```

### `battery()`

Читает состояние батареи.

- Текущая реализация также принимает необязательный `timeout=5`.
- Возвращает `100 - battery_remaining`, то есть процент израсходованного заряда по данным `SYS_STATUS`.

Пример:

```python
battery = await copter.battery()
```

### `read(param, timeout=5)`

Читает MAVLink-параметр автопилота.

- `param` - имя параметра строкой или `bytes`, длина до 16 байт.
- `timeout` - время ожидания `PARAM_VALUE`.
- Возвращает значение параметра.

Пример:

```python
capacity = await copter.read("BATT_CAPACITY")
```

### `write(param, value, param_type=None, timeout=5)`

Записывает MAVLink-параметр автопилота и ожидает подтверждающее значение.

- `param` - имя параметра строкой или `bytes`, длина до 16 байт.
- `value` - новое значение параметра.
- `param_type` - тип MAVLink-параметра. В интерфейсе значение по умолчанию `None`; реализация `Params.write()` по умолчанию использует `MAV_PARAM_TYPE_REAL32`.
- `timeout` - время ожидания `PARAM_VALUE`.
- Возвращает значение параметра, полученное после записи.

Пример:

```python
await copter.write("BATT_CAPACITY", 5200.0)
```

## Методы `XArduCopter`, не входящие в `XCopter`

Эти методы не объявлены в `xcopter.py`, но нужны для практического использования
реализации:

### `connect(connection_string, baud=115200, source_system=255, source_component=0, gcs=False)`

Открывает MAVLink-соединение и ожидает первый heartbeat.

- `connection_string` - строка подключения, например `udp:127.0.0.1:14550`,
  `udpin:0.0.0.0:14550`, `udpout:127.0.0.1:14550`, `tcp:127.0.0.1:5760`
  или `uart:/dev/ttyUSB0`.
- `baud` - скорость UART-соединения.
- `source_system` - MAVLink system id отправителя.
- `source_component` - MAVLink component id отправителя.
- `gcs` - если `True`, соединение работает как GCS и запускает фоновую отправку heartbeat.

Пример:

```python
await copter.connect("udp:127.0.0.1:14550", baud=115200, gcs=True)
await copter.connect("tcp:127.0.0.1:5760", baud=115200)
await copter.connect("uart:/dev/ttyUSB0", baud=115200)
```

### `heartbeat_loop()`

Фоновая задача отправки GCS heartbeat. Обычно не вызывается напрямую.

Пример:

```python
await copter.connect("udp:127.0.0.1:14550", gcs=True)
```

### `pid(P=(0.8, 0.8, 0.8, 0.5), I=(0.0, 0.0, 0.0, 0.0), D=(0.2, 0.2, 0.2, 0.1), pid_loop=10, min_speed=0.1, max_speed=10.0, min_rate=5.0, max_rate=100.0, epsilon=0.05)`

Настраивает коэффициенты и ограничения PID-контроллера, используемого методом
`move()`.

Пример:

```python
copter.pid(P=(0.8, 0.8, 0.8, 0.5), epsilon=0.1)
```

### `target_reached(error, target_hold_time=1.0, now=None)`

Проверяет, удерживается ли ошибка по всем осям в пределах текущего `epsilon`
дольше `target_hold_time`.

Пример:

```python
reached = copter.target_reached((0.01, 0.02, 0.0, 0.5))
```

### `move(move_timeout=300, target_hold_time=1.0, error_fn=None)`

Выполняет PID-движение по ошибке, которую возвращает пользовательская функция
`error_fn`. Функция должна возвращать последовательность из четырех значений:
ошибка по X, Y, Z и yaw.

Пример:

```python
def error_fn():
    return (1.0, 0.0, 0.0, 0.0)


await copter.move(move_timeout=10, error_fn=error_fn)
```

## Логирование

Реализация наследует методы логирования из `Log`:

- `debug(msg)`
- `info(msg)`
- `warn(msg)`
- `error(msg)`
- `critical(msg)`

Они пишут сообщение в стандартный `logging` и, если соединение активно,
отправляют MAVLink `STATUSTEXT`.

Пример:

```python
copter.info("Mission started")
copter.warn("Battery is low")
copter.error("Mission failed")
```
