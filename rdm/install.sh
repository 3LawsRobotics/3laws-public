#!/usr/bin/env bash
SCRIPT_VERSION="0.5.3"

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

promptChoiceInstall() {
  local REPLY=""
  select which in "Docker container" "ROS package"; do
    case $which in
    "Docker container")
      REPLY="docker"
      break
      ;;
    "ROS package")
      REPLY="package"
      break
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
Usage: ${0##*/} [-h] [-y] [-i package|docker] [-s auto|manual] [-t <DOCKER_TOKEN>] [-r <ROBOT_ID>] <COMPANY_ID>
Install 3Laws Robot Diagnostic Module
   -h                 show this help menu
   -y                 answer yes to all yes/no questions
   -i package|docker  install mode
   -s auto|manual     start mode
   -t <DOCKER_TOKEN>  docker api token
   -r <ROBOT_ID>      robot identified
If -i, -s, -t, or -r are not specified, you will be prompted during the installation
EOF
}

# Script Options
ALWAYS_YES=0
INSTALL_MODE=""
START_MODE=""
DOCKER_TOKEN=""
ROBOT_ID=""

while getopts hyi:s:r:t: opt; do
  case $opt in
  h)
    show_help
    exit 0
    ;;
  y)
    ALWAYS_YES=1
    ;;
  i)
    INSTALL_MODE=$(echo "$OPTARG" | tr '[:upper:]' '[:lower:]')
    ;;
  s)
    START_MODE=$(echo "$OPTARG" | tr '[:upper:]' '[:lower:]')
    ;;
  t)
    DOCKER_TOKEN="$OPTARG"
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
if [[ -n $INSTALL_MODE && $INSTALL_MODE != "package" && $INSTALL_MODE != "docker" ]]; then
  cerr "option -s can only be used with either 'package' or 'docker'"
  exit 22
fi
if [[ -n $START_MODE && $START_MODE != "auto" && $START_MODE != "manual" ]]; then
  cerr "option -s can only be used with either 'auto' or 'manual'"
  exit 22
fi

# Define some variables
COMPANY_ID=$1
LAWS3_DIR="$HOME/3lawsRoboticsInc"
SCRIPT_DIR="$LAWS3_DIR/scripts"
PACKAGE_DIR="3laws_$COMPANY_ID"
DOCKER_REGISTRY=ghcr.io/3lawsrobotics
DOCKER_IMAGE_NAME="3laws_rdm_$COMPANY_ID"
DOCKER_IMAGE_LINK="$DOCKER_REGISTRY/$DOCKER_IMAGE_NAME:latest"
DOCKER_CONFIG=(--config "$HOME/.docker/3laws")
USERID=$(id -u)
GROUPID=$(id -g)
BRANCH=master
LOCAL_INSTALL=0
CURRENT_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

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

# Install mode prompt
if [ -z "$INSTALL_MODE" ]; then
  cin "Do you want to install the diagnostic module as:"
  INSTALL_MODE=$(promptChoiceInstall)
fi

if [[ $INSTALL_MODE == "package" ]]; then
  # Package install mode
  cout "Install diagnostic module as a package..."

  # Install dependencies
  STDLIB=libstdc++-11-dev
  STDLIB_INSTALLED=0
  dpkg -l $STDLIB &>/dev/null && STDLIB_INSTALLED=1
  SED_INSTALLED=0
  dpkg -l sed &>/dev/null && SED_INSTALLED=1
  CRON_INSTALLED=0
  dpkg -l cron &>/dev/null && CRON_INSTALLED=1
  GIT_INSTALLED=0
  dpkg -l git &>/dev/null && GIT_INSTALLED=1
  if [[ $STDLIB_INSTALLED == 0 || $SED_INSTALLED == 0 || $CRON_INSTALLED == 0 || $GIT_INSTALLED == 0 ]]; then
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
      cerr "Failed to clone github repo. Make sure you have setup your ssh keys properly and have been given access to the 3laws repo!"
      exit 13
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
        SRVNAME=3laws_rdm_ros2.service
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
        LINE="0 * * * * bash -c 'bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/$BRANCH/rdm/update.sh) PACKAGE $COMPANY_ID $HOME $LAWS3_DIR $(whoami) $USERID $GROUPID \"$ROBOT_ID\" $SRVNAME $ROS_DISTRO 2>&1 | /usr/bin/logger -t 3LAWS_RDM_UPDATE_PACKAGE'"
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
    cout "In that case, please add the following line to your .bashrc:"
    echo -e "  source $PWD/$PACKAGE_DIR/packages/ROS2/local_setup.bash"
    cout "Source your bashrc :"
    echo -e "  source ~/.bashrc"
    cout "And either launch the diagnostic module directly:"
    echo -e "  ros2 launch lll_rdm rdm.launch.py log_level:=info $ROBOT_ID_STR"
    cout "or add the following action to the LaunchDescription in your launch file:"
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

elif [[ "$INSTALL_MODE" == "docker" ]]; then
  # Docker install mode
  cout "Install diagnostic module as a docker container..."

  # Check if DOCKER_TOKEN variable set
  while [ -z "$DOCKER_TOKEN" ]; do
    cin "Please provide your docker token:"
    read -r DOCKER_TOKEN
    if [ -z "$DOCKER_TOKEN" ]; then
      cerr "Docker token cannot be empty!"
    fi
  done

  # Check docker installed
  {
    docker --version &>/dev/null
  } || {
    cerr "Docker not installed on your machine!"
    INSTALL_DOCKER=$(promptYesNo "Do you want us to install docker for you" 1)
    if [[ "$INSTALL_DOCKER" == 1 ]]; then
      {
        cout "Installing docker..."
        $SUDO apt-get update &>/dev/null
        $SUDO apt-get install -y ca-certificates gpg &>/dev/null
        cwarn "Installed 'ca-certificates', and 'gpg' on system!"
        $SUDO install -m 0755 -d /etc/apt/keyrings &>/dev/null
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | $SUDO gpg --batch --yes --dearmor -o /etc/apt/keyrings/docker.gpg &>/dev/null
        $SUDO chmod a+r /etc/apt/keyrings/docker.gpg &>/dev/null
        echo "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" |
          $SUDO tee /etc/apt/sources.list.d/docker.list &>/dev/null

        $SUDO apt-get update &>/dev/null
        $SUDO apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin &>/dev/null

        cout "Docker successfully installed!"
      } || {
        cerr "Failed to install Docker!"
        exit 65
      }
    else
      cerr "Aborting installation"
      exit 65
    fi
  }

  # Check docker compose installed
  {
    docker compose version &>/dev/null
  } || {
    {
      cout "Installing dependencies..."
      $SUDO apt-get update &>/dev/null
      $SUDO apt-get install -y docker-compose-plugin &>/dev/null
      cwarn "Installed 'docker-compose-plugin' on system!"
    } || {
      cerr "Failed to install 'docker-compose-plugin' dependency!"
      exit 65
    }
  }

  cout "Loging into docker registry..."
  {
    echo "$DOCKER_TOKEN" | docker "${DOCKER_CONFIG[@]}" login "$DOCKER_REGISTRY" -u 3lawscustomers --password-stdin &>/dev/null
  } || {
    cerr "Cannot log into docker registry. Is your access token correct?"
    exit 13
  }

  cout "Pulling docker image..."
  {
    docker "${DOCKER_CONFIG[@]}" pull "$DOCKER_IMAGE_LINK"
  } || {
    cerr "Failed to pull docker image!"
    exit 13
  }

  # Setup autostart
  if [ -z "$START_MODE" ]; then
    cin "How do you want to start the module:"
    START_MODE=$(promptChoiceLaunch)
  fi

  if [[ $START_MODE == "auto" ]]; then
    {
      SED_INSTALLED=0
      dpkg -l sed &>/dev/null && SED_INSTALLED=1
      if [[ $SED_INSTALLED == 0 ]]; then
        {
          cout "Installing dependencies..."
          $SUDO apt-get update &>/dev/null
          $SUDO apt-get install -y sed &>/dev/null
          cwarn "Installed 'sed' on system!"
        } || {
          cerr "Failed to install 'sed' dependency!"
          exit 65
        }
      fi

      cout "Installing daemon..."
      SRVNAME=3laws_rdm_docker
      SRVNAME_FULL=$SRVNAME.service
      DOCKER_COMPOSE_NAME=$SRVNAME.docker-compose.yaml
      DOCKER_COMPOSE_PATH=$SCRIPT_DIR/$DOCKER_COMPOSE_NAME

      if [[ $LOCAL_INSTALL == 1 ]]; then
        sed "s+@DOCKER_IMAGE_LINK@+$DOCKER_IMAGE_LINK+g; s+@HOME@+$HOME+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g; s+@LAWS3_ROBOT_ID@+$ROBOT_ID+g" "$CURRENT_SCRIPT_DIR/$DOCKER_COMPOSE_NAME" >"$DOCKER_COMPOSE_PATH"

        sed "s+@DOCKER_COMPOSE_PATH@+$DOCKER_COMPOSE_PATH+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g" "$CURRENT_SCRIPT_DIR/$SRVNAME_FULL" >"$SCRIPT_DIR/$SRVNAME_FULL"

      else
        curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/$BRANCH/rdm/$DOCKER_COMPOSE_NAME" |
          sed "s+@DOCKER_IMAGE_LINK@+$DOCKER_IMAGE_LINK+g; s+@HOME@+$HOME+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g; s+@LAWS3_ROBOT_ID@+$ROBOT_ID+g" >"$DOCKER_COMPOSE_PATH"

        curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/$BRANCH/rdm/$SRVNAME_FULL" |
          sed "s+@DOCKER_COMPOSE_PATH@+$DOCKER_COMPOSE_PATH+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g" > \
            "$SCRIPT_DIR/$SRVNAME_FULL"
      fi

      $SUDO mv -f "$SCRIPT_DIR/$SRVNAME_FULL" "/etc/systemd/system/$SRVNAME_FULL" &>/dev/null
      $SUDO systemctl daemon-reload
      $SUDO systemctl enable "$SRVNAME_FULL" &>/dev/null
      $SUDO systemctl restart "$SRVNAME_FULL" &>/dev/null

      # Cron
      cd "$SCRIPT_DIR"
      cout "Adding cron update job..."
      $SUDO crontab -l >crontmp
      sed -i "/$3LAWS_RDM_UPDATE_DOCKER'$/d" crontmp # Remove previous versions of job
      LINE="30 * * * * bash -c 'bash <(curl -fsSL https://raw.githubusercontent.com/3LawsRobotics/3laws-public/$BRANCH/rdm/update.sh) DOCKER $COMPANY_ID $HOME $LAWS3_DIR $(whoami) $USERID $GROUPID \"$ROBOT_ID\" NO_PULL 2>&1 | /usr/bin/logger -t 3LAWS_RDM_UPDATE_DOCKER'"
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
    echo -e "  $SUDO systemctl status $SRVNAME_FULL"
    cout "To stop the daemon, run:"
    echo -e "  $SUDO systemctl stop $SRVNAME_FULL"
    cout "To start the daemon, run:"
    echo -e "  $SUDO systemctl start $SRVNAME_FULL"
    cout "To deactivate the daemon on boot, run:"
    echo -e "  $SUDO systemctl disable $SRVNAME_FULL"
    cout "To activate the daemon on boot, run:"
    echo -e "  $SUDO systemctl enable $SRVNAME_FULL"

  else
    if [ -n "$ROBOT_ID" ]; then
      ROBOT_ID_STR=" -e LAWS3_ROBOT_ID=\"$ROBOT_ID\""
    fi
    cout "In that case, run the following commands to start the docker container:"
    echo -e "docker run -it --rm --name 3laws_rdm -u $USERID:$GROUPID$ROBOT_ID_STR --net=host --pid=host -v /dev/shm:/dev/shm -v /etc/machine-id:/3laws_robotics/machine-id $DOCKER_IMAGE_LINK"
    cout "If you want to run the container in detached mode:"
    echo -e "docker run -d --rm --name 3laws_rdm -u $USERID:$GROUPID$ROBOT_ID_STR --net=host --pid=host  -v /dev/shm:/dev/shm -v /etc/machine-id:/3laws_robotics/machine-id $DOCKER_IMAGE_LINK"
  fi
else
  cerr "Unkown update mode, should be one of : [PACKAGE, DOCKER]"
  exit 22
fi
