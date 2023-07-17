#!/usr/bin/env bash
SCRIPT_VERSION="0.3.4"

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

promptYesNo() {
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

# Define some variables
LAWS3_DIR="$HOME/3lawsRoboticsInc"

# Main
ctitle "3Laws Robot Diagnostic Module uninstaller (v$SCRIPT_VERSION)"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi

# Cloned repo
if [ -d "${LAWS3_DIR}" ]; then
  cout "Removing '${LAWS3_DIR}' directory..."
  {
    rm -rf "$LAWS3_DIR"
  } || {
    warn "Failed to delete '$LAWS3_DIR' directory. Make sure you have correct write access to this directory!"
  }
fi

remove_service() {
  SERVICE_ACTIVE=0
  $SUDO systemctl cat -- "$1" &>/dev/null && SERVICE_ACTIVE=1
  if [[ "$SERVICE_ACTIVE" == 1 ]]; then
    cout "Removing '$1' daemon..."
    {
      $SUDO systemctl stop "$1" &>/dev/null
      $SUDO systemctl disable "$1" &>/dev/null
      $SUDO rm /etc/systemd/system/"$1"
    } || {
      warn "Failed to remove '$1' daemon, you may want to make sure there are not lingering services running: '$SUDO systemctl status $SRVNAME'!"
    }
  fi
}

# Check if service is running
remove_service 3laws_rdm_ros.service
remove_service 3laws_rdm_docker.service

# Remove cron jobs
cd "$SCRIPT_DIR"
{
  $SUDO crontab -l | sed "/$3LAWS_RDM_UPDATE_PACKAGE'$/d" | sed "/$3LAWS_RDM_UPDATE_DOCKER'$/d" | $SUDO crontab
  $SUDO service cron reload &>/dev/null
} || {
  warn "Failed to remove cron update jobs, you may want to check you cron jobs: '$SUDO crontab -e'"
}

cout "Uninstall finished!"
