FROM espressif/idf:release-v4.4
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get install -yq sudo lsb-release gosu nano && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update -q && apt-get install -yq python3-pip
RUN source $IDF_PATH/export.sh
RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
	
ARG USER_ID=espidf

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG USER_ID
USER $USER_ID

RUN git clone https://github.com/micro-ROS/micro_ros_espidf_component.git /opt/esp/idf/components/micro_ros_espidf_component

# Set default location after container startup.
WORKDIR /ws

ENTRYPOINT ["idf.py", "build", "flash", "monitor"]
