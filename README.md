# `kuc_uld_ajr2025` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
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

# Delete this part if you are using it as a template

ROS 2 pacage template, to get started, use template by clicking on the Green button labeled [`Use this template`](https://github.com/kucserka.szabolcs@hallgato.sze.hu/kuc_uld_ajr2025/generate) / [`Create new repository`](https://github.com/kucserka.szabolcs@hallgato.sze.hu/kuc_uld_ajr2025/generate). 

<p align="center"><img src="img/use_this_template01.png" width="60%" /></p>


Let's assume 
- your Github username is `mycoolusername`
- your ROS 2 repo shold be `cool_ros2_package`

Replace everything in the cloned repo:

- `kuc_uld_ajr2025` >> `cool_ros2_package` (the folder was already renamed after `Use this template`)
- `kucserka.szabolcs@hallgato.sze.hu` >> `mycoolusername`
- find all `todo` strings and fill the blanks

The easiest way is VS code:

<p align="center"><img src="img/replace01.png" width="60%" /></p>

Now `colcon build` your ROS 2 package and you can start wokring.