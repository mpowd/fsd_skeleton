#!/bin/zsh

CATKIN_SHELL=zsh

export FSD_ROOT=$( cd "$(dirname "${ZSH_SOURCE[0]}" )" && pwd)

printf "Sourcing fsd_skeleton/devel/setup.zsh\n"

# check whether devel folder exists
if [ -f "${FSD_ROOT}/devel/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "${FSD_ROOT}/devel/setup.zsh"
else
    source "/opt/ros/noetic/setup.zsh"
    printf "You need to build first before you can source\n"
    printf "Run 'catkin build' in the skeleton_repo directory\n"
fi

source ${FSD_ROOT}/fsd_aliases
