# Oruga microcontroller code

This is a pico-zenoh client for the Oruga's ESP32 microcontroller.

## The robot

Oruga is a differential tracked robot for outdoor use. It is based on the [Lynxmotion A4WD3](https://www.lynxmotion.com/a4wd3-rugged-rovers/) platform and has a custom-built ESP32-based control system.

![Oruga robot](https://github.com/xopxe/pico-oruga-platformio-espidf/blob/main/docs/oruga.jpg?raw=true)

## Electric schematics

The wiring schematics are in the [docs](docs/) directory. As always, the first movements must be performed with the robot "in the air". If the motor's polarities or encoder directions are reversed, the robot can misbehave violently. It is recommended to run with RViz2 to verify that all directions match: physical motors vs reported odometry and joint rotation in RViz2 (see [oruga_ws](https://github.com/xopxe/oruga_ws) for instructions).

## Instalation

* Open this project in VSCode with Platformio extension installed.

* Install additional dependencies:

```sh
cd .pio/libdeps/pico32
git clone https://github.com/Pico-ROS/Pico-ROS-software.git
```

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

Apache 2.0
