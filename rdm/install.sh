#!/usr/bin/env bash
SCRIPT_VERSION="0.6.1"

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

# Usage info
show_help() {
  cat <<EOF
Usage: ${0##*/} [-h] [-y] [-s auto|manual] [-r <ROBOT_ID>] <COMPANY_ID>
Install 3Laws Robot Diagnostic Module
   -h                 show this help menu
   -y                 answer yes to all yes/no questions
   -s auto|manual     start mode
   -r <ROBOT_ID>      robot identified
If -s or -r are not specified, you will be prompted during the installation
EOF
}

# Script Options
ALWAYS_YES=0
START_MODE=""
ROBOT_ID=""

while getopts hys:r: opt; do
  case $opt in
  h)
    show_help
    exit 0
    ;;
  y)
    ALWAYS_YES=1
    ;;
  s)
    START_MODE=$(echo "$OPTARG" | tr '[:upper:]' '[:lower:]')
    ;;
  r)
    ROBOT_ID="$OPTARG"
    ;;
  *)
    show_help >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

# Check options consitancy
if [[ -n $START_MODE && $START_MODE != "auto" && $START_MODE != "manual" ]]; then
  cerr "option -s can only be used with either 'auto' or 'manual'"
  exit 22
fi

# Define some variables
COMPANY_ID=$1
LAWS3_DIR="$HOME/3lawsRoboticsInc"
SCRIPT_DIR="$LAWS3_DIR/scripts"
PACKAGE_DIR="3laws_$COMPANY_ID"
USERID=$(id -u)
GROUPID=$(id -g)
BRANCH=master

# Main
ctitle "3Laws Robot Diagnostic Module Installer (v$SCRIPT_VERSION)"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi

# Create 3laws directory
{
  mkdir -p "$SCRIPT_DIR"
  mkdir -p "$LAWS3_DIR/config"
} || {
  cerr "Cannot create '$LAWS3_DIR' directory, make sure you have write access to your home folder!"
  exit 13
}

# Got to 3laws main directory
cd "$LAWS3_DIR"

# Check if company id set
if [ -z "$COMPANY_ID" ]; then
  cerr "Script must be called with at least one argument : <COMPANY_ID> !"
  exit 22
fi

# Check if ROBOT_ID variable set
if [ -z "$ROBOT_ID" ]; then
  cin "Please provide your robot identifier (or leave empty if you want to define it later):"
  read -r ROBOT_ID
fi

# Check ros versions
HAS_ROS1=0
if command -v roscore &>/dev/null; then
  HAS_ROS1=1
fi
HAS_ROS2=0
if command -v ros2 &>/dev/null; then
  HAS_ROS2=1
fi
if [[ $HAS_ROS1 == 0 && $HAS_ROS2 == 0 ]]; then
  HAS_ROS1=1
  HAS_ROS2=1
fi

# Package install mode
cout "Install diagnostic module as a package..."

# Install dependencies
SED_INSTALLED=0
dpkg -l sed &>/dev/null && SED_INSTALLED=1
CRON_INSTALLED=0
dpkg -l cron &>/dev/null && CRON_INSTALLED=1
GIT_INSTALLED=0
dpkg -l git &>/dev/null && GIT_INSTALLED=1
if [[ $SED_INSTALLED == 0 || $CRON_INSTALLED == 0 || $GIT_INSTALLED == 0 ]]; then
  cout "Installing dependencies..."
  $SUDO apt-get update &>/dev/null
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

if [[ $CRON_INSTALLED == 0 ]]; then
  {
    $SUDO apt-get install -y cron &>/dev/null
    cwarn "Installed 'cron' on system!"
  } || {
    cerr "Failed to install 'cron' dependency!"
    exit 65
  }
fi

if [[ $GIT_INSTALLED == 0 ]]; then
  {
    $SUDO apt-get install -y git &>/dev/null
    cwarn "Installed 'git' on system!"
  } || {
    cerr "Failed to install 'git' dependency!"
    exit 65
  }
fi

# Check if already cloned
CLONE=1
if [ -d "$PACKAGE_DIR" ]; then
  CLONE=$(promptYesNo "It seems that the Diagnostic Module is already installed, do you want to overwrite it" 1)
  if [[ $CLONE == 1 ]]; then
    {
      rm -rf "$PACKAGE_DIR"
    } || {
      cerr "Failed to delete '$PACKAGE_DIR' directory. Make sure you have correct write access to this directory!"
      exit 13
    }
  fi
fi

# Clone repo
if [[ $CLONE == 1 ]]; then
  cout "Cloning 3laws repo..."
  {
    git clone "git@github.com:3LawsRobotics/$PACKAGE_DIR.git" &>/dev/null
  } || {
    {
      cwarn "Failed to clone repo over ssh, trying over https..."
      git clone "https://github.com/3LawsRobotics/$PACKAGE_DIR.git" &>/dev/null
    } || {
      cerr "Failed to clone github repo. Make sure you have setup your ssh keys properly and have been given access to the 3laws repo!"
      exit 13
    }
  }
fi

# Setup autostart
if [ -z "$START_MODE" ]; then
  cin "How do you want to start the module:"
  START_MODE=$(promptChoiceLaunch)
fi
if [[ $START_MODE == "auto" ]]; then
  (
    {
      LLL_WS="$LAWS3_DIR/$PACKAGE_DIR"
      # Systemctl
      SRVNAME=3laws_rdm_ros.service
      cout "Installing daemon..."
      sed "s+@LLL_WS@+$LLL_WS+g; s+@ROS_DISTRO@+$ROS_DISTRO+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g; s+@LAWS3_ROBOT_ID@+$ROBOT_ID+g;" "$LLL_WS/packages/$SRVNAME" \
        >"$LAWS3_DIR/$SRVNAME"
      $SUDO mv -f "$LAWS3_DIR/$SRVNAME" "/etc/systemd/system/$SRVNAME" &>/dev/null
      $SUDO systemctl daemon-reload
      $SUDO systemctl enable $SRVNAME &>/dev/null
      $SUDO systemctl restart $SRVNAME &>/dev/null

      # Cron
      cd "$SCRIPT_DIR"
      cout "Adding cron update job..."
      $SUDO crontab -l >crontmp
      sed -i "/$3LAWS_RDM_UPDATE_PACKAGE'$/d" crontmp # Remove previous versions of job
      LINE="0 * * * * bash -c 'bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/$BRANCH/rdm/update.sh) $COMPANY_ID $HOME $LAWS3_DIR $(whoami) $USERID $GROUPID \"$ROBOT_ID\" $SRVNAME $ROS_DISTRO 2>&1 | /usr/bin/logger -t 3LAWS_RDM_UPDATE_PACKAGE'"
      echo "$LINE" >>crontmp
      $SUDO crontab crontmp
      $SUDO service cron reload &>/dev/null
      rm crontmp

    } || {
      cerr "Failed to install daemon!"
      exit 13
    }
    cout "Daemon installed successfully and activated on boot!"
    cout "To check the daemon status, run:"
    echo -e "  $SUDO systemctl status $SRVNAME"
    cout "To stop the daemon, run:"
    echo -e "  $SUDO systemctl stop $SRVNAME"
    cout "To start the daemon, run:"
    echo -e "  $SUDO systemctl start $SRVNAME"
    cout "To deactivate the daemon on boot, run:"
    echo -e "  $SUDO systemctl disable $SRVNAME"
  )
else # Manual
  if [ -n "$ROBOT_ID" ]; then
    ROBOT_ID_STR=" robot_id:=\"$ROBOT_ID\""
  fi
  cout "To run the diagnostic module directly, execute the following command:"
  echo -e "$LAWS3_DIR/$PACKAGE_DIR/packages/start_rdm.sh log_level:=info$ROBOT_ID_STR"
  echo -e " "
  if [[ $HAS_ROS1 == 1 ]]; then
    cout "If you want to start it as part of your ROS1 launch system, add the following line to your .bashrc:"
    echo -e "  source $LAWS3_DIR/$PACKAGE_DIR/packages/3laws_setup.sh"
    cout "Source your bashrc :"
    echo -e "  source ~/.bashrc"
    cout "Add the following action to the LaunchDescription in your launch file:"
    echo -e "  <include file=\"\$(find lll_rdm)/launch/rdm.launch\">"
    echo -e "    <arg name=\"log_level\" value=\"info\"/>"
    if [ -n "$ROBOT_ID" ]; then
      echo -e "    <arg name=\"robot_id\" value=\"$ROBOT_ID\"/>"
    fi
    echo -e "  </include>"
    echo -e " "
  fi
  if [[ $HAS_ROS2 == 1 ]]; then
    cout "If you want to start it as part of your ROS2 launch system, add the following line to your .bashrc:"
    echo -e "  source $LAWS3_DIR/$PACKAGE_DIR/packages/3laws_setup.sh"
    cout "Source your bashrc :"
    echo -e "  source ~/.bashrc"
    cout "Add the following action to the LaunchDescription in your launch file:"
    echo -e "  from launch.actions import IncludeLaunchDescription"
    echo -e "  from launch.launch_description_sources import PythonLaunchDescriptionSource"
    echo -e "  from launch.substitutions import PathJoinSubstitution"
    echo -e " "
    echo -e "  IncludeLaunchDescription("
    echo -e "      PythonLaunchDescriptionSource("
    echo -e "          PathJoinSubstitution("
    echo -e "              ["
    echo -e "                  get_package_share_directory('lll_rdm'),"
    echo -e "                  'launch',"
    echo -e "                  'rdm.launch.py',"
    echo -e "              ]"
    echo -e "          )"
    echo -e "      ),"
    echo -e "      launch_arguments={"
    echo -e "          'log_level': 'info',"
    if [ -n "$ROBOT_ID" ]; then
      echo -e "          'robot_id': '$ROBOT_ID',"
    fi
    echo -e "      }.items(),"
    echo -e "  )"
    echo -e " "
  fi
  cout "For more information see your readme: https://github.com/3LawsRobotics/3laws_$COMPANY_ID/blob/$BRANCH/README.md"
fi
