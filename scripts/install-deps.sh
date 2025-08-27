# Assets and models

curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash # Install git LFS

ROS_DISTRO=noetic
sudo apt update

sudo apt install libgeographic-dev

sudo apt install ros-$ROS_DISTRO-geographic-msgs

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

sudo apt install libdw-dev
sudo apt install libncurses5-dev libncursesw5-dev

