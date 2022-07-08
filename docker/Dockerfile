FROM ros:galactic-ros-core

RUN apt-get -y update && apt-get install -y --no-install-recommends \
        iputils-ping \
        python3-rosdep2 \
        python3-colcon-common-extensions \
        build-essential \
        ros-galactic-gscam \
        vim \
        ros-galactic-joy \
        ros-galactic-rqt-image-view \
#        git \
#        minicom \
#        python3-pip \
#        python3-catkin-tools \
#        python3-wstool \
#        python3-dev \
#        ssh \
#        git-lfs \
#        geographiclib-tools \
    && apt-get clean && apt-get autoremove && rm -rf /var/lib/apt/lists/*

COPY blueye_ros2_interface /root/blueyews/src/blueye_ros2_interace
COPY blueye_controls_gui_plugin /root/blueyews/src/blueye_control_gui_plugin
COPY blueye_ros2_msgs /root/blueyews/src/blueye_ros2_msgs
COPY ros2_shared /root/blueyews/src/ros2_shared

WORKDIR /root/blueyews
RUN apt-get -y update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r \
    && apt-get clean && apt-get autoremove && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/galactic/setup.sh \ 
    && colcon build

COPY run_blueye.sh /root/blueyews/run_blueye.sh

ENTRYPOINT [ "/root/blueyews/run_blueye.sh" ]