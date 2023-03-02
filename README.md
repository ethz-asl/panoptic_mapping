![Ubuntu 18 + ROS Melodic](https://github.com/ethz-asl/panoptic_mapping/actions/workflows/build_test_18.yml/badge.svg) ![Ubuntu 20 + ROS Noetic](https://github.com/ethz-asl/panoptic_mapping/actions/workflows/build_test_20.yml/badge.svg) ![Docker](https://github.com/ethz-asl/panoptic_mapping/actions/workflows/docker-publish.yml/badge.svg)

# Panoptic Mapping
This package contains **panoptic_mapping**, a general framework for semantic volumetric mapping. We provide, among other, a submap-based approach that leverages panoptic scene understanding towards adaptive spatio-temporally consistent volumetric mapping, as well as regular, monolithic semantic mapping.

![combined](https://user-images.githubusercontent.com/36043993/135645102-e5798e36-e2b0-4611-9260-ec9d54d38e47.png)

Multi-resolution 3D Reconstruction, active and inactive panoptic submaps for temporal consistency, online change detection, and more.

# Table of Contents
**Credits**
* [Paper](#Paper)
* [Video](#Video)

**Setup**
* [Installation](#Installation)
* [Datasets](#Datasets)

**Examples**
- [Running the Panoptic Mapper](#running-the-panoptic-mapper)
- [Monolithic Semantic Mapping](#monolithic-semantic-mapping)
- [Running the RIO Dataset](#running-the-rio-dataset)

**Other**
* [Contributing](#Contributing)

# Paper
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Jeffrey Delmerico, Johannes Schönberger, Juan Nieto, Marc Pollefeys, Roland Siegwart, and Cesar Cadena. "**Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency**" in *IEEE International Conference on Robotics and Automation (ICRA)*, pp. 8018-8024, 2022.
  \[ [IEEE](https://ieeexplore.ieee.org/document/9811877) | [ArXiv](https://arxiv.org/abs/2109.10165) | [Video](https://www.youtube.com/watch?v=A7o2Vy7_TV4) \]
  ```bibtex
  @inproceedings{schmid2022panoptic,
    title={Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency},
    author={Schmid, Lukas and Delmerico, Jeffrey and Sch{\"o}nberger, Johannes and Nieto, Juan and Pollefeys, Marc and Siegwart, Roland and Cadena, Cesar},
    booktitle={2022 IEEE International Conference on Robotics and Automation (ICRA)},
    year={2022},
    volume={},
    number={},
    pages={8018-8024},
    doi={10.1109/ICRA46639.2022.9811877}}
  }
  ```

# Video
For a short overview explaining the approach check out our video on youtube:

[<img src="https://user-images.githubusercontent.com/36043993/155131772-60757e47-c458-4f5b-9c6b-a6e6cbca8976.jpg" alt="youtube video">](https://www.youtube.com/watch?v=A7o2Vy7_TV4)

# Installation
Instructions for different installation options. The repository was developed and tested on Ubuntu 18.04 with ROS melodic and Ubuntu 20.04 with ROS noetic.

**Docker**
To run the panoptic mapper without installation, just use the docker image:
```bash
docker pull ghcr.io/ethz-asl/panoptic_mapping:main
docker run -it ghcr.io/ethz-asl/panoptic_mapping:main bash
```

**VSCode**
For development with vscode, you can just clone this repository and follow the prompts to open it in a [devcontainer](https://code.visualstudio.com/docs/devcontainers/containers).

**System Installation**
To install on linux, follow the instructions below matching your OS and ROS version:

<details>
  <summary>Ubuntu 18.04 + ROS Melodic.</summary>
<p>

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

  ```shell script
  sudo apt-get install python-catkin-tools
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  catkin config --extend /opt/ros/melodic
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  catkin config --merge-devel
  ```

**Installation**

1. Install system dependencies:

  ```shell script
  sudo apt-get install python-wstool python-catkin-tools autoconf libtool git
  ```

2. Move to your catkin workspace:

  ```shell script
  cd ~/catkin_ws/src
  ```

3. Download repo using [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh) or HTTPS:

  ```shell script
  git clone git@github.com:ethz-asl/panoptic_mapping.git  # SSH
  git clone https://github.com/ethz-asl/panoptic_mapping.git  # HTTPS
  ```

4. Download and install package dependencies using ros install:

  * If you created a new workspace.

  ```shell script
  wstool init . ./panoptic_mapping/panoptic_mapping_ssh.rosinstall    # SSH
  wstool init . ./panoptic_mapping/panoptic_mapping_https.rosinstall  # HTTPS
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
</p>
</details>

<details>
<summary>Ubuntu 20.04 + ROS Noetic.</summary>
<p>

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

  ```shell script
  sudo apt-get install python3-catkin-tools
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  catkin config --extend /opt/ros/noetic
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  catkin config --merge-devel
  ```

**Installation**

1. Install system dependencies:

  ```shell script
  sudo apt-get install python3-pip python3-wstool python3-catkin-tools autoconf libtool git
  pip3 install osrf-pycommon
  ```

2. Move to your catkin workspace:

  ```shell script
  cd ~/catkin_ws/src
  ```

3. Download repo using [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh) or HTTPS:

  ```shell script
  git clone git@github.com:ethz-asl/panoptic_mapping.git  # SSH
  git clone https://github.com/ethz-asl/panoptic_mapping.git  # HTTPS
  ```

4. Download and install package dependencies using ros install:

  * If you created a new workspace.

  ```shell script
  wstool init . ./panoptic_mapping/panoptic_mapping_ssh.rosinstall    # SSH
  wstool init . ./panoptic_mapping/panoptic_mapping_https.rosinstall  # HTTPS
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
</p>
</details>


# Datasets
The datasets described in the paper and used for the demo can be downloaded from the [ASL Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping).

A utility script is provided to directly download the flat data:
```
roscd panoptic_mapping_utils/scripts
chmod +x download_flat_dataset.sh
./download_flat_dataset.sh "/home/$USER/data"  # Or whichever destination you prefer.
```

To run the RIO demos, the [original dataset](https://waldjohannau.github.io/RIO/) needs to be downloaded and augmented with [our supplementary data](https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping). Instructions on which scenes to download and how to combine them are found on our dataset page.

# Examples
## Running the Panoptic Mapper
This example explains how to run the Panoptic Multi-TSDF mapper on the flat dataset.

1. If not already done so, download the flat dataset as explained [here](#datasets).

2. Replace the data `base_path` in `launch/run.launch (L10)` and `file_name` in `config/mapper/flat_groundtruth.yaml (L15)` to the downloaded path.
3. Run the mapper:
    ```
    roslaunch panoptic_mapping_ros run.launch
    ```
4. You should now see the map being incrementally built:

    <img src="https://user-images.githubusercontent.com/36043993/135860249-6334cc41-5758-457b-8f65-b017e2905804.png" width="400">

5. After the map finished building, you can save the map:
    ```
    rosservice call /panoptic_mapper/save_map "file_path: '/path/to/run1.panmap'"
    ```
6. Terminate the mapper pressing Ctrl+C. You can continue the experiment on `run2` of the flat dataset by changing the `base_path`-ending in `launch/run.launch (L10)` to `run2`, and `load_map` and `load_path` in `launch/run.launch (L26-27)` to `true` and `/path/to/run1.panmap`, respectively. Optionally, you can also change the `color_mode` in `config/mapper/flat_groundtruth.yaml (L118)` to `change` to better highlight the change detection at work.
     ```
    roslaunch panoptic_mapping_ros run.launch
    ```
7. You should now see the map being updated based on the first run:

    <img src="https://user-images.githubusercontent.com/36043993/135861611-4d576750-3104-4d73-87dc-60b7a4ad1df6.png" width="400">

## Monolithic Semantic Mapping
Panoptic Mapping supports also the monolithic use case. This example explains how to run the Panoptic Single-TSDF mapper on the flat dataset.

1. If not already done so, download the flat dataset as explained [here](#datasets).

2. Replace the data `base_path` in `launch/run.launch (L10)` and `file_name` in `config/mapper/single_tsdf.yaml (L15)` to the downloaded path.

3. To use the single-TSDF mapper and real segmentation predictions, set `use_detectron` in `launch/run.launch (L6)` to true and `config` in `launch/run.launch (L22)` to 'single_tsdf'.

4. Run the mapper:

   ```
   roslaunch panoptic_mapping_ros run.launch
   ```

5. You should now see the map being incrementally built.

6. Varying visualization modes are supported, the `classes` (default) will color the mesh according to the predicted semantic class. Other modes, such as `classification` will show the confidence of the aggregated predictions.

![single_combined](https://user-images.githubusercontent.com/36043993/143055994-2703f57c-cf4a-4126-a43c-cb7f47b6da02.png)

Predicted classes (left) and corresponding fusion confidence (right, low to high in red to green).

## Running the RIO Dataset
This example explains how to run the Panoptic Multi-TSDF mapper on the RIO dataset.

1. First, download the [original dataset](https://waldjohannau.github.io/RIO/) and [our supplementary data](https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping).

2. Set `use_rio` in `launch/run.launch (L5)` to true, replace the `data_path` in `(L16)` and `file_name` in `config/mapper/rio_groundtruth.yaml (L15)` to the downloaded path.

3. Run the mapper:
    ```
    roslaunch panoptic_mapping_ros run.launch
    ```
4. You should now see the map being incrementally built:

    <img src="https://user-images.githubusercontent.com/36043993/157041619-3f683ee7-2709-4bc5-84c2-8b1188ee52c5.png" width="800">

    (Left in reconstructed color, right colored by submap)


# Contributing
**panoptic_mapping** is an open-source project, any contributions are welcome!

For issues, bugs, or suggestions, please open a [GitHub Issue](https://github.com/ethz-asl/panoptic_mapping/issues).

To add to this repository:
* Please employ the [feature-branch workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).
* Setup our auto-formatter for coherent style (we follow the [google style guide](https://google.github.io/styleguide/cppguide.html)):
    ```
    # Download the linter
    cd <linter_dest>
    git clone git@github.com:ethz-asl/linter.git -b feature/noetic
    cd linter
    echo ". $(realpath setup_linter.sh)" >> ~/.bashrc
    bash
    roscd panoptic_mapping/..
    init_linter_git_hooks
    # You're all set to go!
    ```
* Please open a [Pull Request](https://github.com/ethz-asl/panoptic_mapping/pulls) for your changes.
* Thank you for contributing!
