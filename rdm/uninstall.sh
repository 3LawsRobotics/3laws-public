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

SCRIPT_VERSION="v0.3.0"
LAWS3_DIR="${HOME}/3lawsRoboticsInc"

# Exit on errors
set -e

# Main
echo -e "${BWhite}${On_Green}3Laws Robot Diagnostic Module Uninstaller (${SCRIPT_VERSION})${NC}"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi

# Cloned repo
if [ -d "${LAWS3_DIR}" ]; then
  cout "Removing '${LAWS3_DIR}' directory..."
  {
    rm -rf $LAWS3_DIR
  } || {
    warn "Failed to delete `${LAWS3_DIR}` directory. Make sure you have correct write access to this directory!"
  }
fi

function remove_service()
{
  SERVICE_ACTIVE=0
  $SUDO systemctl cat -- ${SRVNAME} &> /dev/null && SERVICE_ACTIVE=1
  if [[ "$SERVICE_ACTIVE" == 1 ]]; then
    cout "Removing '${SRVNAME}' daemon..."
    {
      $SUDO systemctl stop ${SRVNAME} &> /dev/null
      $SUDO systemctl disable ${SRVNAME} &> /dev/null
      $SUDO rm /etc/systemd/system/${SRVNAME}
    } || {
      warn "Failed to remove `${SRVNAME}` daemon, sorry about that!"
    }
  fi
}

# Check if service is running
SRVNAME=3laws_rdm_ros2.service
remove_service

SRVNAME=3laws_rdm_docker.service
remove_service

cout "Uninstall successful!"
