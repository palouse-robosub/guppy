sudo apt update
sudo apt upgrade -y

sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization \
    ros-jazzy-ros2-control \
    ros-jazzy-pluginlib \
    ros-jazzy-xacro \
    ros-jazzy-control-toolbox \
    ros-jazzy-proxsuite \
    ros-jazzy-marine-acoustic-msgs \
    python3-colcon-common-extensions \
    python3-evdev \
    python3-rosdep \
    python3-pygame \
    nlohmann-json3-dev

sudo python3 -m pip install PySide6 --break-system-packages

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

sudo apt install -y git neovim python3-pip terminator
sudo snap install code --classic

cd ~
git clone https://github.com/PalouseRobosub/guppy --recurse-submodules -j8
