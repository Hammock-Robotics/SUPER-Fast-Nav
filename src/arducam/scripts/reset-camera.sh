#!/usr/bin/env bash
set -e

echo "[INFO] Reloading uvcvideo kernel module..."

# remove uvcvideo
sudo /sbin/modprobe -r uvcvideo

# add uvcvideo
sudo /sbin/modprobe uvcvideo

echo "[INFO] uvcvideo reloaded successfully."
