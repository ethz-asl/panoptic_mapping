# Panoptic Mapping
A panoptic, submap-based approach towards volumetric spatio-temporally consistent mapping for robot interaction.

![combined](https://user-images.githubusercontent.com/36043993/110769139-2ab26780-8258-11eb-8b7a-ed4f2e050ea4.png)

## Table of Contents
* [Credits](#Credits)
* [Installation](#Installation)
* [Demo](#Demo)

# Credits
* **Author:** Lukas Schmid <schmluk@mavt.ethz.ch>
* **Affiliation:** Autonomous Systems Lab (ASL), ETH Zürich.
* **Version:** Pre-release Beta
* **License:** The project is work in progress and not yet licensed or open-sourced. By working with this project you agree to keep any code, ideas, or other, confidential until the project is published.


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

# Examples
## Demo
Uses the `run1.bag` dataset, ask the authors for access. This dataset contains sythetic color, depth and segmentation images for an indoor scene.
Update line 5 of `panoptic_mapping_ros/config/demo_mapper.yaml` to the destination of the downloaded labels file.

Then launch the demo by running: 
```
roslaunch panoptic_mapping_ros demo.launch bag_name:=/path/to/run1.bag
```

An RVIZ window should show up visualizing the scene being reconstructed, where each object lives in it's own submap with varying resolutions.

The submaps, representing the segmentation, can be visualized by changing the coloring:
```
rosservice call /panoptic_mapper/set_visualization_mode "visualization_mode: '' color_mode: 'submaps'" 
```
![combined](https://user-images.githubusercontent.com/36043993/110769139-2ab26780-8258-11eb-8b7a-ed4f2e050ea4.png)
