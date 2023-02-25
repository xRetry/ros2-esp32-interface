#FROM espressif/idf:latest
FROM microros/esp-idf-microros:latest

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
#RUN apt-get -y install cmake
#
#RUN . ${IDF_PATH}/export.sh
#RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions
RUN git clone https://github.com/micro-ROS/micro_ros_espidf_component.git /opt/esp/idf/components/micro_ros_espidf_component

# Set default location after container startup.
WORKDIR /ws
