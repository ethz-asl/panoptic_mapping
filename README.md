# Panoptic Mapping
This package contains **panoptic_mapping**, a submap-based approach that leverages panoptic scene understanding towards adaptive spatio-temporally consistent volumetric mapping.

![combined](https://user-images.githubusercontent.com/36043993/110769139-2ab26780-8258-11eb-8b7a-ed4f2e050ea4.png)

## Table of Contents
* [Credits](#Credits)
* [Installation](#Installation)
* **To come soon:**
  * Video
  * Download the datasets.
  * Run a demo.
  * Monolithic Mapper.
  * Contributing.

# Credits
* **Author:** Lukas Schmid <schmluk@mavt.ethz.ch>
* **Affiliation:** Autonomous Systems Lab (ASL), ETH Zürich.
* **Version:** Pre-release Gamma
* **License:** The project is work in progress and not yet licensed or open-sourced. It will be upon publication. By working with this project you agree to keep any code, ideas, or other, confidential until the project is published.


# Installation
Installation instructions for Linux.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):
    ```shell script    
    # Create a new workspace
    sudo apt-get install python-catkin-tools
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin config --extend /opt/ros/$ROS_DISTRO
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin config --merge-devel
    ```

**Installation**

1. Install system dependencies:
    ```shell script
    sudo apt-get install python-wstool python-catkin-tools
    ```

2. Move to your catkin workspace:
    ```shell script
    cd ~/catkin_ws/src
    ```

3. Download repo using [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh):
    ```shell script
    git clone git@github.com:ethz-asl/panoptic_mapping.git -b release/alpha
    ```

4. Download and install package dependencies using ros install:
    * If you created a new workspace.
    ```shell script
    wstool init . ./panoptic_mapping/panoptic_mapping.rosinstall
    wstool update
    ```

    * If you use an existing workspace. Notice that some dependencies require specific branches that will be checked out.
    ```shell script
    wstool merge -t . ./panoptic_mapping/panoptic_mapping.rosinstall
    wstool update
    ```

5. Compile and source:
    ```shell script
    catkin build panoptic_mapping_utils
    source ../devel/setup.bash
    ```
