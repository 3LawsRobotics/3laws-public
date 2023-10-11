# 3laws-public

## Robot diagnostic module installation:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/install.sh) [-h (help)] [-y (yes to all)] [-s auto|manual (start mode)] [-r <ROBOT_ID>] <COMPANY_ID>
```
If `-s` or `-r` are not specified, you will be prompted during the installation.

## Robot diagnostic module update:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/update.sh)
```

## Robot diagnostic module uninstall:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/uninstall.sh)
```

## Ros2 node and topic discovery script:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/ros_graph_discovery.sh)
```
