# 3laws-public

## Robot diagnostic module installation:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/install.sh) [-h (help)] [-i package|docker (install mode)] [-s auto|manual (start mode)] [-t <DOCKER_TOKEN> (docker api token)] <COMPANY_ID>
```
If `-i` or `-s` are not specified, you will be prompted during the installation. `-t` is only required if using `-i docker`.

## Robot diagnostic module uninstall:
```bash
bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/uninstall.sh)
```
