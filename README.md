# `kuc_uld_ajr2025` package
ROS 2 C++ package ami 2 nodeból áll, és a Turtlesim segítségével rajzol egy gúlát.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/kucserka.szabolcs@hallgato.sze.hu/kuc_uld_ajr2025
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select kuc_uld_ajr2025 --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch kuc_uld_ajr2025 launch_example1.launch.py
```

