#!/bin/bash

# Exit on errors
set -e

HAS_ROS1=0
if command -v rostopic list &>/dev/null; then
  HAS_ROS1=1
fi
HAS_ROS2=0
if command -v ros2 topic list &>/dev/null; then
  HAS_ROS2=1
fi

# shellcheck disable=SC2129
# Get the list of topics
if [ "$HAS_ROS1" -eq 1 ]; then
  topicsRos1=$(rostopic list)

  echo "starting scan"
  echo "======= TOPICS ========" >ros1_network_info.txt
  echo >>ros1_network_info.txt
  # Iterate through each topic
  for topic in $topicsRos1; do
    echo "exploring topic: $topic"
    echo "Topic: $topic" >>ros1_network_info.txt
    echo "-----------------" >>ros1_network_info.txt

    # Get the info for the current topic
    rostopic info "$topic" >>ros1_network_info.txt

    echo "=================" >>ros1_network_info.txt
  done

  echo >>ros1_network_info.txt
  echo >>ros1_network_info.txt
  echo "======= NODES ========" >>ros1_network_info.txt
  echo >>ros1_network_info.txt
  # Get the list of topics
  nodes=$(rosnode list)

  # Iterate through each topic
  for node in $nodes; do
    echo "exploring node: $node"
    echo "Node: $node" >>ros1_network_info.txt
    echo "-----------------" >>ros1_network_info.txt

    # Get the info for the current topic
    rosnode info "$node" >>ros1_network_info.txt

    echo "=================" >>ros1_network_info.txt
  done
  echo "End of ROS1 scan, results saved in ros1_network_info.txt"
else
  echo "ROS1 not found, passing to ROS2"
fi

# shellcheck disable=SC2129
# Get the list of ros2 topics
if [ "$HAS_ROS2" -eq 1 ]; then
  topicsRos2=$(ros2 topic list) &>/dev/null

  echo "======= TOPICS ========" >ros2_network_info.txt
  echo >>ros2_network_info.txt
  # Iterate through each topic
  for topic in $topicsRos2; do
    echo "exploring topic: $topic"
    echo "Topic: $topic" >>ros2_network_info.txt
    echo "-----------------" >>ros2_network_info.txt

    # Get the info for the current topic
    ros2 topic info -v "$topic" >>ros2_network_info.txt

    echo "=================" >>ros2_network_info.txt
  done

  echo >>ros2_network_info.txt
  echo >>ros2_network_info.txt
  echo "======= NODES ========" >>ros2_network_info.txt
  echo >>ros2_network_info.txt
  # Get the list of topics
  nodes=$(ros2 node list)

  # Iterate through each topic
  for node in $nodes; do
    echo "exploring node: $node"
    echo "Node: $node" >>ros2_network_info.txt
    echo "-----------------" >>ros2_network_info.txt

    # Get the info for the current topic
    ros2 node info "$node" >>ros2_network_info.txt

    echo "=================" >>ros2_network_info.txt
  done

  echo "End of ROS2 scan, results saved in ros2_network_info.txt"
else
  echo "ROS2 not found"
fi
