#FROM microros/esp-idf-microros:latest
FROM espressif/idf:release-v4.4

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt update -q && \

    apt install -yq sudo lsb-release gosu nano && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

COPY ./install_micro_ros_deps_script.sh /install_micro_ros_deps_script.sh

RUN mkdir -p /tmp/install_micro_ros_deps_script && mv /install_micro_ros_deps_script.sh /tmp/install_micro_ros_deps_script/ && \

    /tmp/install_micro_ros_deps_script/install_micro_ros_deps_script.sh && \
    rm -rf /var/lib/apt/lists/*
	
ARG USER_ID=espidf

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \

    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG USER_ID
USER $USER_ID

WORKDIR /

RUN apt-get -y update

# SET UP DEV ENV

RUN apt-get -y install curl git python3 python3-pip g++ tmux software-properties-common unzip

# Set bash settings
RUN echo "set -o vi" >> /root/.bashrc
RUN echo "export TERM=screen-256color-bce" >> /root/.bashrc

# Get tmux config
RUN curl https://raw.githubusercontent.com/xRetry/arch-setup/main/roles/packages/files/tmux.conf > /root/.tmux.conf

# Install neovim
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim.appimage
RUN chmod u+x nvim.appimage
RUN ./nvim.appimage --appimage-extract
RUN ln -s /squashfs-root/AppRun /usr/bin/nvim

# Install packer
RUN git clone --depth 1 https://github.com/wbthomason/packer.nvim /root/.local/share/nvim/site/pack/packer/start/packer.nvim

# Create directory for neovim configuration files.
RUN mkdir -p /root/.config/nvim
RUN git clone https://github.com/xRetry/nvim.git /root/.config/nvim

# Install neovim extensions.
RUN nvim --headless +PackerSync +'sleep 10' +qall

RUN nvim --headless +'TSInstall c cpp' +'MasonInstall clangd' +'sleep 10' +qall

# Create directory for projects (there should be mounted from host).
RUN mkdir -p /ws

# Project specific installs
RUN git clone https://github.com/micro-ROS/micro_ros_espidf_component.git /opt/esp/idf/components/micro_ros_espidf_component
#RUN git clone https://github.com/xRetry/ros2-esp32-interfaces.git /opt/esp/idf/components/micro_ros_espidf_component/extra_packages/ros2-esp32-interfaces

#RUN apt-get -y install bundler
#RUN git clone --recursive https://github.com/throwtheswitch/cmock.git
#WORKDIR /cmock
#RUN bundle install 
RUN gem install ceedling

# Set default location after container startup.
WORKDIR /ws
