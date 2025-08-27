#!/usr/bin/env bash
set -e

USER=thurdparty
SUDOERS_FILE="/etc/sudoers.d/reset-camera"

echo "[INFO] Adding sudoers rule for $USER in $SUDOERS_FILE..."

# Safely write sudoers file
sudo tee "$SUDOERS_FILE" >/dev/null <<EOF
$USER ALL=(ALL) NOPASSWD: /sbin/modprobe -r uvcvideo, /sbin/modprobe uvcvideo
EOF

# Set correct permissions
sudo chmod 440 "$SUDOERS_FILE"

echo "[INFO] Done. $USER can now run reset-camera.sh without password."
