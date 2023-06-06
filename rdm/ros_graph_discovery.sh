#!/bin/bash


echo "======= TOPICS ========" > ros_network_info.txt
echo >> ros_network_info.txt
echo "starting scan"

# Get the list of topics
topics=$(ros2 topic list)

# Iterate through each topic
for topic in $topics
do
  echo "exploring topic: $topic"
  echo "Topic: $topic" >> ros_network_info.txt
  echo "-----------------" >> ros_network_info.txt

  # Get the info for the current topic
  ros2 topic info -v $topic >> ros_network_info.txt

  echo "=================" >> ros_network_info.txt
done

echo >> ros_network_info.txt
echo >> ros_network_info.txt
echo "======= NODES ========" >> ros_network_info.txt
echo >> ros_network_info.txt
# Get the list of topics
nodes=$(ros2 node list)

# Iterate through each topic
for node in $nodes
do
  echo "exploring node: $node"
  echo "Node: $node" >> ros_network_info.txt
  echo "-----------------" >> ros_network_info.txt

  # Get the info for the current topic
  ros2 node info $node >> ros_network_info.txt

  echo "=================" >> ros_network_info.txt
done

echo "End of scan, results saved in ros_network_info.txt"