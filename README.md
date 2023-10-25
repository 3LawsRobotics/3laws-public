# 3laws-public

## Robot diagnostic module installation:

### Interactive Package
```bash
sudo bash -c "$(wget -qO - https://raw.githubusercontent.com/3LawsRobotics/3laws-public/beta/rdm/install.sh)"
```

### Specific package

Download the package:
```bash
wget https://raw.githubusercontent.com/3LawsRobotics/3laws-public/beta/rdm/install.sh
```
Make it executable
```bash
chmod +x install.sh
```
Run it with your arguments
```bash
sudo ./install.sh [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>]
```

if ```-yf -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION>``` specified, the script is non interactive

## Robot diagnostic module uninstall:

#### For Ros2:
```bash
sudo dpkg --remove "lll-rdm-$ROS_DISTRO"
```

#### For Ros1:
```bash
sudo dpkg --remove "lll-rdm-$(rosversion)"
```

## Ros2 node and topic discovery script:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/ros_graph_discovery.sh)
```
