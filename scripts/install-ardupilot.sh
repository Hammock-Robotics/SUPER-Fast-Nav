
sudo apt update
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git config --global url.https://.insteadOf git://
git checkout Copter-4.5.5
git submodule update --init --recursive
