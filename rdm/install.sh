#!/usr/bin/env bash
SCRIPT_VERSION="0.7.0"

# Exit on errors
set -e

# Colors
NC='\033[0m'          # Text Reset
On_Black='\033[40m'   # Black background
On_Purple='\033[45m'  # Purple background
On_Cyan='\033[46m'    # Cyan background
On_Green='\033[42m'   # Green background
BIRed='\033[1;91m'    # Bold Intense Red
BWhite='\033[1;37m'   # Bold White
BIYellow='\033[1;93m' # Bold Intense Yellow
White='\033[0;37m'    # White

# Printing utilities
ColErr=${BIRed}${On_Black}
ColPrompt=${BWhite}${On_Cyan}
ColInfo=${White}${On_Purple}
ColWarn=${BIYellow}${On_Black}
cout() {
  echo -e "${ColInfo}${1}${NC}"
}
cin() {
  echo -e "${ColPrompt}${1}${NC}"
}
cerr() {
  echo -e "${ColErr}${1}${NC}"
}
cwarn() {
  echo -e "${ColWarn}${1}${NC}"
}
ctitle() {
  echo -e "${BWhite}${On_Green}${1}${NC}"
}

# Local functions
promptYesNo() {
  if [[ $ALWAYS_YES == 1 ]]; then
    echo 1
    return
  fi

  local REPLY=${2}
  local TXT="[y/n]"

  if [[ "$REPLY" == 1 ]]; then
    TXT="[Y/n]"
  elif [[ "$REPLY" == 0 ]]; then
    TXT="[y/N]"
  else
    REPLY=""
  fi

  while true; do
    local color=$'\033[1;37m\033[46m'
    local noColor=$'\033[0m'
    read -r -e -p "$color${1} $TXT?$noColor " yn

    case $yn in
    y | Y | Yes | yes | YES)
      REPLY=1
      break
      ;;
    n | N | No | no | NO)
      REPLY=0
      break
      ;;
    "")
      if [ -n "$REPLY" ]; then
        break
      fi
      ;;
    *) ;;
    esac
  done

  echo "$REPLY"
}

promptChoiceLaunch() {
  local REPLY=""
  select which in "Automatic" "Manual"; do
    case $which in
    "Automatic")
      REPLY="auto"
      break
      ;;
    "Manual")
      REPLY="manual"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

promptChoiceArch() {
  local REPLY=""
  select which in "amd64" "arm64"; do
    case $which in
    "amd64")
      REPLY="amd64"
      break
      ;;
    "arm64")
      REPLY="arm64"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

promptChoiceUbuntuDistro() {
  local REPLY=""
  select which in "22.04" "20.04" "18.04"; do
    case $which in
    "22.04")
      REPLY="22.04"
      break
      ;;
    "20.04")
      REPLY="20.04"
      break
      ;;
    "18.04")
      REPLY="18.04"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

promptChoiceROSDistro() {
  local REPLY=""
  select which in "iron" "humble" "foxy" "galactic" "noetic" "melodic"; do
    case $which in
    "iron")
      REPLY="iron"
      break
      ;;
    "humble")
      REPLY="humble"
      break
      ;;
    "foxy")
      REPLY="foxy"
      break
      ;;
    "galactic")
      REPLY="galactic"
      break
      ;;
    "noetic")
      REPLY="noetic"
      break
      ;;
    "melodic")
      REPLY="melodic"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

valid_args() {

  if [[ $UBUNTU_DISTRO == "22.04" ]]; then
    local valid_ros=("iron" "humble")

    if [[ " ${valid_ros[@]} " =~ " ${QUERY_DISTRO} " ]]; then
      echo "1"
    else
      echo "0"
    fi
  fi
  if [[ $UBUNTU_DISTRO == "20.04" ]]; then
    local valid_ros=("foxy" "noetic" "galactic")

    if [[ " ${valid_ros[@]} " =~ " ${QUERY_DISTRO} " ]]; then
      echo "1"
    else
      echo "0"
    fi
  fi
  if [[ $UBUNTU_DISTRO == "18.04" ]]; then
    local valid_ros=("melodic")

    if [[ " ${valid_ros[@]} " =~ " ${QUERY_DISTRO} " ]]; then
      echo "1"
    else
      echo "0"
    fi
  fi
}

# Usage info
show_help() {
  cat <<EOF
Usage: ${0##*/} [-h] [-y] [-s auto|manual] [-r <ROBOT_ID>] <COMPANY_ID>
Install 3Laws Robot Diagnostic Module
   -h                 show this help menu
   -y                 answer yes to all yes/no questions
   -r <ROBOT_ID>      robot identified
If -r is not specified, the robot name can be specified in the configuration file or using the variable ROBOT_ID
EOF
}

check_values() {
  if [[ -z $ARCH ]]; then
    cerr "Architecture not found, specify amd64|arm64"
    ARCH=$(promptChoiceArch)
  fi
  if [[ -z $UBUNTU_DISTRO ]]; then
    cerr "Ubuntu distro not found, specify 22.04|20.04|18.04"
    UBUNTU_DISTRO=$(promptChoiceUbuntuDistro)
  fi
  if [[ -z $QUERY_DISTRO ]]; then
    cerr "ROS distro not found, specify iron|humble|foxy|galactic|noetic|melodic"
    echo $QUERY_DISTRO
    QUERY_DISTRO=$(promptChoiceROSDistro)
  fi

  if [[ $(valid_args) == 0 ]]; then
    cerr "ROS distro not compatible with your ubuntu distibution"
    echo "22.04: iron | humble"
    echo "20.04: noetic | galactic | foxy"
    echo "18.04: melodic"
    cout "Specify iron|humble|foxy|galactic|noetic|melodic"
    QUERY_DISTRO=$(promptChoiceROSDistro)
  fi

  cout "The RDM package for ubuntu $UBUNTU_DISTRO $ARCH and Ros $QUERY_DISTRO will be downloaded"
}

# Script Options
ALWAYS_YES=0

while getopts hy opt; do
  case $opt in
  h)
    show_help
    exit 0
    ;;
  y)
    ALWAYS_YES=1
    ;;
  *)
    show_help >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

# Main
ctitle "3Laws Robot Diagnostic Module Installer (v$SCRIPT_VERSION)"

HAS_ROS1=0
if command -v roscore &>/dev/null; then
  HAS_ROS1=1
  ROS1_DISTRO=$(rosversion -d)
  QUERY_DISTRO=$ROS1_DISTRO
fi
HAS_ROS2=0
if command -v ros2 &>/dev/null; then
  HAS_ROS2=1
  QUERY_DISTRO=$ROS_DISTRO
fi
if [[ $HAS_ROS1 == 0 && $HAS_ROS2 == 0 ]]; then
  HAS_ROS1=1
  HAS_ROS2=1
  USE_ROS2=$(promptYesNo "ROS1 and ROS2 detected ! Do you want to use ROS2 as the RDM interface ?")
  if [[ $USE_ROS2 == 0 ]]; then
    cout "ROS1 will be used as RDM interface with distro $ROS1_DISTRO"
    QUERY_DISTRO=$ROS1_DISTRO
  else
    cout "ROS2 will be used as RDM interface with distro $ROS_DISTRO"
    HAS_ROS1=0
    QUERY_DISTRO=$ROS_DISTRO
  fi
fi

UBUNTU_DISTRO=$(cat /etc/*-release | grep VERSION_ID | grep -oE "[0-9]{2}.[0-9]{2}")

ARCH=amd64
case "$(uname -i)" in
arm* | aarch*)
  ARCH=arm64
  ;;
esac

# Check args validity
check_values

REGEX_QUERY="lll-rdm-${QUERY_DISTRO}_[0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]\+_$ARCH"

# Define variables.
GH_API="https://api.github.com"
GH_REPO="$GH_API/repos/3LawsRobotics/3laws"
GH_TAGS="$GH_REPO/releases/latest"
CURL_ARGS="-LJO#"
# GITHUB_API_TOKEN=TOKEN
# AUTH="Authorization: token $GITHUB_API_TOKEN"

curl -o /dev/null -s $GH_REPO || {
  echo "Error: Invalid repo, token or network issue!"
  exit 1
}

# Read asset tags.
RESPONSE=$(curl -s -H "application/vnd.github+json" $GH_TAGS)
ASSET_NAME=$(echo "$RESPONSE" | grep -o "name.:.\+${REGEX_QUERY}.deb" | cut -d ":" -f2- | cut -d "\"" -f2-)
ASSET_ID=$(echo "$RESPONSE" | grep -C3 "name.:.\+$REGEX_QUERY" | grep -w id | tr : = | tr -cd '[[:alnum:]]=' | cut -d'=' -f2-)

[ "$ASSET_ID" ] || {
  VALID_ASSETS=$(echo "$RESPONSE" | grep -o "name.:.\+lll-rdm-[a-zA-Z]\+_[0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]_[a-zA-Z0-9]\+" | cut -d ":" -f2- | cut -d "\"" -f2-)
  if [ -z "$VALID_ASSETS" ]; then
    cerr "An error occured, please contact support@3lawsrobotics.com"
  else
    echo -e "Error: Failed to get asset id, valid packages:\n$VALID_ASSETS"
  fi
  exit 1
}

GH_ASSET="$GH_REPO/releases/assets/$ASSET_ID"

DOWNLOAD=$(promptYesNo "Do you want to download $ASSET_NAME in the current directory" 1)
if [[ $DOWNLOAD == 1 ]]; then
  echo "Downloading package..." >&2
  curl $CURL_ARGS -s -H 'Accept: application/octet-stream' "$GH_ASSET"
  cout "Package $ASSET_NAME has been downloaded."

  # Install dependencies
  STDLIB=libstdc++-113-dev
  STDLIB_INSTALLED=0
  dpkg -l $STDLIB &>/dev/null && STDLIB_INSTALLED=1
  SED_INSTALLED=0
  dpkg -l sed &>/dev/null && SED_INSTALLED=1

  if [[ $STDLIB_INSTALLED == 0 || $SED_INSTALLED == 0 ]]; then
    cout "Installing dependencies..."
    $SUDO apt-get update &>/dev/null
  fi

  if [[ $STDLIB_INSTALLED == 0 ]]; then
    {
      {
        $SUDO apt-get install -y $STDLIB &>/dev/null
      } || {
        $SUDO apt-get install -y --no-install-recommends software-properties-common &>/dev/null
        $SUDO add-apt-repository -y "ppa:ubuntu-toolchain-r/test" &>/dev/null
        cwarn "Added 'ppa:ubuntu-toolchain-r/test' to apt sources!"
        $SUDO apt-get install -y $STDLIB &>/dev/null
      }
      cwarn "Installed '$STDLIB' on system!"
    } || {
      cerr "Failed to install '$STDLIB' dependency!"
      exit 65
    }
  fi

  if [[ $SED_INSTALLED == 0 ]]; then
    {
      $SUDO apt-get install -y sed &>/dev/null
      cwarn "Installed 'sed' on system!"
    } || {
      cerr "Failed to install 'sed' dependency!"
      exit 65
    }
  fi

  # Install package
  echo -e "To install it:\n\tsudo apt install -f ./$ASSET_NAME"
  echo "Configuration files can be found at /opt/3lawsRobotics/config"
else
  cwarn "Package not downloaded."
  cout "If you encounter any issues, please contact: support@3lawsrobotics.com"
fi
