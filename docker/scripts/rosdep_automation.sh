#!/bin/bash
set -e

echo "Running rosdep update and rosdep install for mounted workspace..."

rosdep update
rosdep install --from-paths /ros/home_base_ws/src --ignore-src -r -y

exec "$@"
