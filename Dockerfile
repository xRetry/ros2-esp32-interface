FROM espressif/idf:release-v4.4
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get install -yq sudo lsb-release gosu nano

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

COPY ./install_micro_ros_deps_script.sh /install_micro_ros_deps_script.sh

RUN mkdir -p /tmp/install_micro_ros_deps_script && mv /install_micro_ros_deps_script.sh /tmp/install_micro_ros_deps_script/ && \
    /tmp/install_micro_ros_deps_script/install_micro_ros_deps_script.sh && \
    rm -rf /var/lib/apt/lists/*

#RUN apt-get update -q && apt-get install -yq python3-pip
#RUN source $IDF_PATH/export.sh
#RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
	
ARG USER_ID=espidf

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG USER_ID
USER $USER_ID
#USER root

RUN git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git /opt/esp/idf/components/micro_ros_espidf_component
RUN git clone https://github.com/xRetry/ros2-esp32-messages.git /opt/esp/idf/components/micro_ros_espidf_component/extra_packages/ros2-esp32-messages

# Set default location after container startup.
WORKDIR /ws
