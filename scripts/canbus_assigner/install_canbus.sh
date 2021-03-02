#!/bin/bash
# This script installs the canbus_assigner.py daemon on machines
# running a standard OctoPi installation of Klipper.

# Force script to exit if an error occurs
set -e

# Find canbus_assigner directory from the pathname of this script
CANDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Install default canbus_assigner.service
sudo cp ${CANDIR}/canbus_assigner.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable canbus_assigner.service
sudo systemctl restart canbus_assigner.service
