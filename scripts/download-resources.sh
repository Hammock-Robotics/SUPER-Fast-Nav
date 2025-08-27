#!/bin/bash
set -e  # exit if any command fails

FILE_ID="1hCXxZ30zXdRYL2hj5VvPbqYhYO14UEgw"
OUTPUT="models.zip"
TARGET_DIR="src/iq_sim/models"

# Check if gdown is installed
if ! command -v gdown &> /dev/null; then
    echo "gdown not found, installing..."
    pip install --quiet gdown
fi

# Download the file
echo "Downloading models.zip from Google Drive..."
gdown "https://drive.google.com/uc?id=${FILE_ID}" -O "${OUTPUT}"

# Make sure target dir exists
mkdir -p "${TARGET_DIR}"

# Unzip into target dir
echo "Extracting ${OUTPUT} to ${TARGET_DIR}..."
unzip -o "${OUTPUT}" -d "${TARGET_DIR}"

# Clean up
echo "Cleaning up..."
rm -f "${OUTPUT}"

echo "Done! Models are in ${TARGET_DIR}"
