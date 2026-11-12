# trossen_widowx_driver_crospi Package

This package contains a driver for controlling the Trossen WidowX using crospi_core, based on template_driver_crospi package which uses ROS2 plugins.

## Installation

This driver requires the [interfaces package](https://github.com/Robotics-Research-Group-KUL/trossen_widowx_interfaces) and the [Trossen C++ driver](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html) (`libtrossen_arm`). The Trossen C++ driver is a CMake package with a *package.xml*, so both can simply be cloned into your workspace and built with *colcon*:

```bash
cd <your_ws>/src
git clone https://github.com/Robotics-Research-Group-KUL/trossen_widowx_interfaces.git
git clone https://github.com/TrossenRobotics/trossen_arm.git
cd <your_ws>
colcon build --packages-up-to trossen_widowx_driver_crospi
```

Alternatively, install the Trossen C++ driver system-wide as described in its [documentation](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html) (`cmake` + `make install`); CMake then finds it with `find_package(libtrossen_arm)`.

Note: Trossen officially supports the C++ driver on Ubuntu 20.04, 22.04 and 24.04 (it is distributed as a prebuilt static library).

## Documentation

For documentation and tutorials, checkout our [crospi website](https://crospi-website-907f83.pages.gitlab.kuleuven.be/).

## License and acknowledgements

Published under the [Apache-2.0 license](LICENSE).

## Authors
<p float="left">
<a href="https://www.kuleuven.be/english/kuleuven/">
    <img src="./resources/logo_kuleuven.png" alt="KU Leuven Logo" width="150"/>
</a>
<a href="https://www.mech.kuleuven.be/en/research/ram">
    <img src="./resources/logo_RAM.png" alt="RAM Logo" width="110" />
</a>
</p>

(c) 2025, KU Leuven, Department of Mechanical Engineering, ROB-Group:

- [Federico Ulloa Rios](https://www.kuleuven.be/wieiswie/en/person/00141400)
