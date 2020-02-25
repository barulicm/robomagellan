#! /usr/bin/env bash

set -e
echo "Installing robomagellan udev rules... "
sudo cp $(rospack find robomagellan_bringup)/udev/99-robomagellan.rules /etc/udev/rules.d/
sudo service udev restart
echo "Done!"
