sudo apt install mono-complete
curl 
h#!/usr/bin/env bash
set -e  # Exit immediately if a command fails
set -o pipefail

# 1. Install prerequisites: curl, gnupg, ca-certificates
echo "Installing prerequisites..."
sudo apt update
sudo apt install -y curl gnupg ca-certificates unzip

# 2. Add the official Mono repository and install mono-complete
echo "Adding Mono repository and installing Mono..."
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | \
    sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install -y mono-complete

# 3. Create directory and download Mission Planner
MP_DIR="$HOME/mission_planner"
echo "Creating Mission Planner directory at $MP_DIR"
mkdir -p "$MP_DIR"
cd "$MP_DIR"
echo "Downloading MissionPlanner-latest.zip..."
curl -LO https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip

# 4. Unzip and set permissions
echo "Unzipping..."
unzip -o MissionPlanner-latest.zip
chmod -R u+rx .

# 5. Optional: Fix for Ubuntu 22.04 missing libdl.so issue
if [[ "$(lsb_release -rs)" == "22.04" ]]; then
  echo "Detected Ubuntu 22.04 — applying libdl.so symlink workaround..."
  sudo ln -sf /usr/lib/x86_64-linux-gnu/libdl.so.2 /usr/lib/x86_64-linux-gnu/libdl.so
fi

# 6. Add current user to dialout group for serial/telemetry access
echo "Adding $USER to dialout group for serial permissions..."
sudo usermod -aG dialout "$USER"
echo "You may need to log out and log back in for group changes to take effect."

# 7. Create a simple launch script
LAUNCH_SH="$MP_DIR/run_mission_planner.sh"
echo "Creating launch script at $LAUNCH_SH"
cat << 'EOF' > "$LAUNCH_SH"
#!/usr/bin/env bash
cd "$(dirname "$0")"
echo "Starting Mission Planner..."
MONO_LOG_LEVEL=warn mono MissionPlanner.exe
EOF
chmod +x "$LAUNCH_SH"

echo
echo "Installation complete!"
echo "To run Mission Planner, execute:"
echo "  $LAUNCH_SH"
echo
echo "Note: Some features (e.g., log analyzer, SITL, joystick support) are known to have limitations under Mono on Linux."  
ttps://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip
