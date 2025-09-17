# Oruga microcontroller code

This is a pico-zenoh client for the Oruga's ESP32 microcontorller.

## Instalation

* Open this roject in VSCode with Platformio extension installed.

* Install aditional dependencies:

```sh
cd PROJECT_DIR
cd .pio/libdeps/pico32
git clone https://github.com/Pico-ROS/Pico-ROS-software.git

# get updated versions of files from repos
cd PROJECT_DIR
mkdir -p config/ucdr
cp .pio/libdeps/pico32/Pico-ROS-software/thirdparty/config/ucdr/config.h config/ucdr/config.h

```

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

Apache 2.0
