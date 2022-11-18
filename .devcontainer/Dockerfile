FROM ghcr.io/ethz-asl/panoptic_mapping:main
WORKDIR /catkin_ws/src/
#RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
#RUN sudo apt install -y ccache && echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc && source ~/.bashrc && echo $PATH
#RUN ccache --max-size=2G
#RUN catkin build maplab
RUN git config --global --add safe.directory /catkin_ws/src/panoptic_mapping
RUN echo 'source /catkin_ws/devel/setup.bash' | tee -a ~/.bashrc
