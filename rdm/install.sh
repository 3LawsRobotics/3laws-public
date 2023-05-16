#!/usr/bin/env bash

# Colors
NC='\033[0m'       # Text Reset
On_Black='\033[40m'       # Black background
On_Purple='\033[45m'      # Purple background
On_Cyan='\033[46m'        # Cyan background
On_Green='\033[42m'       # Green background
BIRed='\033[1;91m'        # Bold Intense Red
BWhite='\033[1;37m'       # Bold White
BIYellow='\033[1;93m'     # Bold Intense Yellow
White='\033[0;37m'        # White


# Colors
ColErr=${BIRed}${On_Black}
ColPrompt=${BWhite}${On_Cyan}
ColInfo=${White}${On_Purple}
ColWarn=${BIYellow}${On_Black}

# Printing utilities
function cout()
{
  echo -e "${ColInfo}${1}${NC}"
}

function cin()
{
  echo -e "${ColPrompt}${1}${NC}"
}

function cerr()
{
  echo -e "${ColErr}${1}${NC}"
}
function cwarn()
{
  echo -e "${ColWarn}${1}${NC}"
}

# Local functions
function promptYesNo()
{
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
    read -e -p "${color}${1} ${TXT}?${noColor} " yn

    case $yn in
      y|Y|Yes|yes|YES)
        REPLY=1
        break
      ;;
      n|N|No|no|NO)
        REPLY=0
        break
      ;;
      "")
        if [ ! -z $REPLY ]; then
          break
        fi
      ;;
      * )
      ;;
    esac
  done

  echo "$REPLY"
}

function promptChoiceInstall()
{
  local REPLY=""
  select which in "Docker container" "ROS package"; do
    case $which in
      "Docker container")
        REPLY="DOCKER"
        break
      ;;
      "ROS package")
        REPLY="PACKAGE"
        break
      ;;
      *)
      ;;
    esac
  done
  echo "$REPLY"
}

function promptChoiceLaunchPackage()
{
  local REPLY=""
  select which in "Automatic" "Launchfile" "Manual"; do
    case $which in
      "Automatic")
        REPLY="DAEMON"
        break
      ;;
      "Launchfile")
        REPLY="LAUNCHFILE"
        break
      ;;
      "Manual")
        REPLY="MANUAL"
        break
      ;;
      *)
      ;;
    esac
  done
  echo "$REPLY"
}

function promptChoiceLaunchDocker()
{
  local REPLY=""
  select which in "Automatic" "Manual"; do
    case $which in
      "Automatic")
        REPLY="DAEMON"
        break
      ;;
      "Manual")
        REPLY="MANUAL"
        break
      ;;
      *)
      ;;
    esac
  done
  echo "$REPLY"
}

# Define some variables
SCRIPT_VERSION="v0.3.0"
COMPANY_ID=$1
DOCKER_TOKEN=$2
LAWS3_DIR="${HOME}/3lawsRoboticsInc"
PACKAGE_DIR="3laws_${COMPANY_ID}"
DOCKER_REGISTRY=ghcr.io/3lawsrobotics
DOCKER_IMAGE_NAME="3laws_rdm_${COMPANY_ID}"
DOCKER_IMAGE_LINK="${DOCKER_REGISTRY}/${DOCKER_IMAGE_NAME}:latest"
DOCKER_CONFIG="--config ${HOME}/.docker/3laws"
USERID=$(id -u)
GROUPID=$(id -g)

# Exit on errors
set -e

# Main
echo -e "${BWhite}${On_Green}3Laws Robot Diagnostic Module Installer (${SCRIPT_VERSION})${NC}"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi


# Create 3laws directory
{
  mkdir -p $LAWS3_DIR/scripts
} || {
  cerr "Cannot create `${LAWS3_DIR}` directory, make sure you have write access to your home folder!"
  exit -1
}

(
  cd $LAWS3_DIR

  # Check if company id set
  if [ -z "$COMPANY_ID" ]; then
    cerr "COMPANY_ID variable not defined!"
    exit -1
  fi

  # Install mode prompt
  cin "Do you want to install the diagnostic module as:"
  INSTALL_MODE=$(promptChoiceInstall)


  if [[ "$INSTALL_MODE" == "PACKAGE" ]]; then
  # Package install mode
    cout "Install diagnostic module as a package..."

    # Install dependencies
    STDLIB=libstdc++-11-dev
    STDLIB_INSTALLED=0
    dpkg -l $STDLIB &> /dev/null && STDLIB_INSTALLED=1
    SED_INSTALLED=0
    dpkg -l sed &> /dev/null && SED_INSTALLED=1
    CRON_INSTALLED=0
    dpkg -l cron &> /dev/null && CRON_INSTALLED=1
    GIT_INSTALLED=0
    dpkg -l git &> /dev/null && GIT_INSTALLED=1
    if [[ "$STDLIB_INSTALLED" == 0 || "$SED_INSTALLED" == 0  || "$CRON_INSTALLED" == 0  || "$GIT_INSTALLED" == 0 ]]; then
      cout "Installing dependencies..."
      $SUDO apt-get update &> /dev/null
    fi

    if [[ "$STDLIB_INSTALLED" == 0 ]]; then
      {
        {
          $SUDO apt-get install -y ${STDLIB} &> /dev/null
        } || {
          $SUDO apt-get install -y --no-install-recommends software-properties-common &> /dev/null
          $SUDO add-apt-repository -y "ppa:ubuntu-toolchain-r/test" &> /dev/null
          cwarn "Added 'ppa:ubuntu-toolchain-r/test' to apt sources!"
          $SUDO apt-get install -y ${STDLIB} &> /dev/null
        }
        cwarn "Installed '${STDLIB}' on system!"
      } || {
        cerr "Failed to install '${STDLIB}' dependency!"
        exit -1
      }
    fi

    if [[ "$SED_INSTALLED" == 0 ]]; then
      {
        $SUDO apt-get install -y sed &> /dev/null
        cwarn "Installed 'sed' on system!"
      } || {
        cerr "Failed to install 'sed' dependency!"
        exit -1
      }
    fi

    if [[ "$CRON_INSTALLED" == 0 ]]; then
      {
        $SUDO apt-get install -y cron &> /dev/null
        cwarn "Installed 'cron' on system!"
      } || {
        cerr "Failed to install 'cron' dependency!"
        exit -1
      }
    fi

    if [[ "$GIT_INSTALLED" == 0 ]]; then
      {
        $SUDO apt-get install -y git &> /dev/null
        cwarn "Installed 'git' on system!"
      } || {
        cerr "Failed to install 'git' dependency!"
        exit -1
      }
    fi

    # Check if already cloned
    CLONE=1
    if [ -d "${PACKAGE_DIR}" ]; then
      CLONE=$(promptYesNo "It seems that the Diagnostic Module is already installed, do you want to overwrite it" 1)
      if [[ "$CLONE" == 1 ]]; then
        {
          rm -rf $PACKAGE_DIR
        } || {
          cerr "Failed to delete `${PACKAGE_DIR}` directory. Make sure you have correct write access to this directory!"
          exit -1
        }
      fi
    fi

    # Clone repo
    if [[ "$CLONE" == 1 ]]; then
      cout "Cloning 3laws repo..."
      {
        git clone "git@github.com:3LawsRobotics/${PACKAGE_DIR}.git" &> /dev/null
      } || {
        cerr "Failed to clone github repo. Make sure you have setup your ssh keys properly and have been given access to the 3laws repo!"
        exit -1
      }
    fi

    # Setup autostart
    cin "How do you want to start the module:"
    LAUNCH_MODE=$(promptChoiceLaunchPackage)
    if [[ "$LAUNCH_MODE" == "DAEMON" ]]; then
    (
      {
        # Systemctl
        SRVNAME=3laws_rdm_ros2.service
        cout "Installing daemon..."
        cd $LAWS3_DIR/$PACKAGE_DIR/rdm
        sed "s+@LAWS3_WS@+$(pwd)+g; s+@ROS_DISTRO@+${ROS_DISTRO}+g; s+@USERID@+${USERID}+g; s+@GROUPID@+${GROUPID}+g" ${SRVNAME} \
          > $LAWS3_DIR/${SRVNAME}
        $SUDO mv -f $LAWS3_DIR/${SRVNAME} /etc/systemd/system/${SRVNAME} &> /dev/null
        $SUDO systemctl enable ${SRVNAME} &> /dev/null
        $SUDO systemctl restart ${SRVNAME} &> /dev/null

        # Cron
        cd $LAWS3_DIR/scripts
        curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/update.sh" > update.sh
        chmod +x update.sh

        # Remove previous versions of job
        cout "Adding cron update job..."
        $SUDO crontab -l > crontmp
        sed -i "/$3LAWS_RDM_UPDATE$/d" crontmp
        LINE="0 * * * * ${LAWS3_DIR}/scripts/update.sh PACKAGE ${COMPANY_ID} $(whoami) ${LAWS3_DIR} ${PACKAGE_DIR} ${SRVNAME} 2>&1 | /usr/bin/logger -t 3LAWS_RDM_UPDATE"
        echo "${LINE}" >> crontmp
        $SUDO crontab crontmp
        $SUDO service cron reload &> /dev/null
        rm crontmp

      } || {
        cerr "Failed to install daemon!"
        exit -1
      }
      cout "Daemon installed successfully and activated on boot!"
      cout "To check the daemon status, run:"
      echo -e "  $SUDO systemctl status ${SRVNAME}"
      cout "To stop the daemon, run:"
      echo -e "  $SUDO systemctl stop ${SRVNAME}"
      cout "To start the daemon, run:"
      echo -e "  $SUDO systemctl start ${SRVNAME}"
      cout "To deactivate the daemon on boot, run:"
      echo -e "  $SUDO systemctl disable ${SRVNAME}"
    )
    elif [[ "$LAUNCH_MODE" == "LAUNCHFILE" ]]; then
      cout "In that case, please add the following line to your .bashrc:"
      echo -e "  export LD_LIBRARY_PATH=${PWD}/${PACKAGE_DIR}/rdm/backend_lib/lib:\$LD_LIBRARY_PATH"
      echo -e "  source ${PWD}/${PACKAGE_DIR}/rdm/ros_packages/local_setup.bash"
      cout "Source your bashrc :"
      echo -e "  source ~/.bashrc"
      cout "Add the following action to the LaunchDescription in your launch file:"
      echo -e "  from launch.actions import IncludeLaunchDescription"
      echo -e "  from launch.launch_description_sources import PythonLaunchDescriptionSource"
      echo -e "  from launch.substitutions import PathJoinSubstitution"
      echo -e " "
      echo -e "  PythonLaunchDescriptionSource("
      echo -e "      PathJoinSubstitution("
      echo -e "          ["
      echo -e "              get_package_share_directory('lll_rdm'),"
      echo -e "              'launch',"
      echo -e "              'rdm.launch.py',"
      echo -e "          ]"
      echo -e "      )"
      echo -e "  )"


    else # Manual
      cout "In that case, please add the following line to your .bashrc:"
      echo -e "  export LD_LIBRARY_PATH=${PWD}/${PACKAGE_DIR}/rdm/backend/lib:\$LD_LIBRARY_PATH"
      echo -e "  source ${PWD}/${PACKAGE_DIR}/rdm/ROS2/local_setup.bash"
      cout "Source your bashrc :"
      echo -e "  source ~/.bashrc"
      cout "Launch diagnostic module:"
      echo -e "  ros2 launch lll_rdm rdm.launch.py "
    fi

  else
  # Docker install mode
    cout "Install diagnostic module as a docker container..."

    # Check if DOCKER_TOKEN variable set
    if [ -z "$DOCKER_TOKEN" ]; then
      cerr "DOCKER_TOKEN variable not defined, cannot pull docker!"
      exit -1
    fi

    # Check docker installed
    {
      docker --version &> /dev/null
    } || {
      cerr "Docker not installed on your machine!"
      INSTALL_DOCKER=$(promptYesNo "Do you want us to install docker for you" 1)
      if [[ "$INSTALL_DOCKER" == 1 ]]; then
        {
          cout "Installing docker..."
          $SUDO apt-get update &> /dev/null
          $SUDO apt-get install -y ca-certificates gpg &> /dev/null
          cwarn "Installed 'ca-certificates', and 'gpg' on system!"
          $SUDO install -m 0755 -d /etc/apt/keyrings &> /dev/null
          curl -fsSL https://download.docker.com/linux/ubuntu/gpg | $SUDO gpg --batch --yes --dearmor -o /etc/apt/keyrings/docker.gpg &> /dev/null
          $SUDO chmod a+r /etc/apt/keyrings/docker.gpg &> /dev/null
          echo \
            "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
            "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
            $SUDO tee /etc/apt/sources.list.d/docker.list &> /dev/null

          $SUDO apt-get update &> /dev/null
          $SUDO apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin &> /dev/null

          cout "Docker successfully installed!"
        } || {
          cerr "Failed to install Docker!"
          exit -1
        }
      else
        exit -1
      fi
    }

    # Check docker compose installed
    {
      docker compose version &> /dev/null
    } || {
      {
        cout "Installing dependencies..."
        $SUDO apt-get update &> /dev/null
        $SUDO apt-get install -y docker-compose-plugin &> /dev/null
        cwarn "Installed 'docker-compose-plugin' on system!"
      } || {
        cerr "Failed to install 'docker-compose-plugin' dependency!"
        exit -1
      }
    }

    cout "Loging into docker registry..."
    {
      echo $DOCKER_TOKEN | docker ${DOCKER_CONFIG} login ${DOCKER_REGISTRY} -u 3lawscustomers --password-stdin &> /dev/null
    } || {
      cerr "Cannot log into docker registry. Is your access token correct?"
      exit -1
    }

    cout "Pulling docker image..."
    {
      docker ${DOCKER_CONFIG} pull $DOCKER_IMAGE_LINK
    } || {
      cerr "Failed to pull docker image!"
      exit -1
    }

    # Setup autostart
    cin "How do you want to start the module:"
    LAUNCH_MODE=$(promptChoiceLaunchDocker)
    if [[ "$LAUNCH_MODE" == "DAEMON" ]]; then
      {
        SED_INSTALLED=0
        dpkg -l sed &> /dev/null && SED_INSTALLED=1
        if [[ "$SED_INSTALLED" == 0 ]]; then
          {
            cout "Installing dependencies..."
            $SUDO apt-get update &> /dev/null
            $SUDO apt-get install -y sed &> /dev/null
            cwarn "Installed 'sed' on system!"
          } || {
            cerr "Failed to install 'sed' dependency!"
            exit -1
          }
        fi

        cout "Installing daemon..."
        SRVNAME=3laws_rdm_docker
        SRVNAME_FULL=${SRVNAME}.service
        COMPOSENAME=${SRVNAME}.docker-compose.yaml
        DOCKER_COMPOSE_LINK=$LAWS3_DIR/scripts/${COMPOSENAME}

        curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/${COMPOSENAME}" | \
          sed "s+@DOCKER_IMAGE_LINK@+${DOCKER_IMAGE_LINK}+g; s+@HOME@+${HOME}+g" > ${DOCKER_COMPOSE_LINK}

        curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/master/rdm/${SRVNAME_FULL}" | \
          sed "s+@DOCKER_COMPOSE_LINK@+${DOCKER_COMPOSE_LINK}+g; s+@USERID@+${USERID}+g; s+@GROUPID@+${GROUPID}+g" > \
          $LAWS3_DIR/scripts/${SRVNAME_FULL}

        $SUDO mv -f $LAWS3_DIR/scripts/${SRVNAME_FULL} /etc/systemd/system/${SRVNAME_FULL} &> /dev/null
        $SUDO systemctl enable ${SRVNAME_FULL} &> /dev/null
        $SUDO systemctl restart ${SRVNAME_FULL} &> /dev/null
      } || {
        cerr "Failed to install daemon!"
        exit -1
      }
      cout "Daemon installed successfully and activated on boot!"
      cout "To check the daemon status, run:"
      echo -e "  $SUDO systemctl status ${SRVNAME_FULL}"
      cout "To stop the daemon, run:"
      echo -e "  $SUDO systemctl stop ${SRVNAME_FULL}"
      cout "To start the daemon, run:"
      echo -e "  $SUDO systemctl start ${SRVNAME_FULL}"
      cout "To deactivate the daemon on boot, run:"
      echo -e "  $SUDO systemctl disable ${SRVNAME_FULL}"
      cout "To activate the daemon on boot, run:"
      echo -e "  $SUDO systemctl enable ${SRVNAME_FULL}"

    else
      cout "In that case, run the following commands to start the docker container:"
      echo -e "docker run -it --rm --name 3laws_rdm -v /etc/machine-id:/3laws_robotics/machine-id -v /proc/stat:/3laws_robotics/proc/stat $DOCKER_IMAGE_LINK"
      cout "If you want to run the container in detached mode:"
      echo -e "docker run -d --rm --name 3laws_rdm -v /etc/machine-id:/3laws_robotics/machine-id -v /proc/stat:/3laws_robotics/proc/stat $DOCKER_IMAGE_LINK"
    fi

  fi
)
