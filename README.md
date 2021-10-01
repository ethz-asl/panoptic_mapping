# Panoptic Mapping
This package contains **panoptic_mapping**, a submap-based approach that leverages panoptic scene understanding towards adaptive spatio-temporally consistent volumetric mapping.

![combined](https://user-images.githubusercontent.com/36043993/135645102-e5798e36-e2b0-4611-9260-ec9d54d38e47.png)

3D Reconstruction, active and inactive panoptic submaps, change detection, and more.

# Table of Contents
**Credits**
* [Paper](#Paper)
* [Video](#Video)

**Setup**
* [Installation](#Installation)
* [Datasets](#Datasets)

**Examples**
* [Running the Panoptic Mapper](#Running-the-Panoptic-Mapper)
* [Monolithic Semantic Mapping](#Monolithic-Semantic-Mapping)



# Paper
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Jeffrey Delmerico, Johannes Schönberger, Juan Nieto, Marc Pollefeys, Roland Siegwart, and Cesar Cadena. "**Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency**" arXiv preprint arXiv:2109.10165 (2021).
\[[ArXiv](https://arxiv.org/abs/2010.09859)\]
  ```bibtex
  @ARTICLE{schmid2021panoptic,
    title={Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency},
    author={Schmid, Lukas and Delmerico, Jeffrey and Sch{\"o}nberger, Johannes and Nieto, Juan and Pollefeys, Marc and Siegwart, Roland and Cadena, Cesar},
    journal={arXiv preprint arXiv:2109.10165},
    year={2021}
  }
  ```
  
# Video
A short video overview explaining the approach will be released upon publication.

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
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
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
    git clone git@github.com:ethz-asl/panoptic_mapping.git
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
# Datasets
The datasets described in the paper and used for the demo can be downloaded from the [ASL Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping).

Additional data to run the mapper on the 3RScan dataset will follow.


# Examples
## Running the Panoptic Mapper

## Monolithic Semantic Mapping
This example will follow shortly.
