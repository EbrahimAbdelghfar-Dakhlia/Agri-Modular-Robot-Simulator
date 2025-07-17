#!/bin/bash
## source ros noetic
source /opt/ros/jazzy/setup.bash
## add commands above to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /etc/bash_completion" >> ~/.bashrc
## run command
exec "$@"