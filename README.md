# 3laws-public

## Robot diagnostic module installation:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/beta/rdm/install.sh) [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>]
```

## Robot diagnostic module uninstall:
```bash
sudo apt pruge lll-rdm-<ROS_DISTRO>
```

## Ros2 node and topic discovery script:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/ros_graph_discovery.sh)
```
