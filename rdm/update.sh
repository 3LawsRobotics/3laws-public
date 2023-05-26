#!/usr/bin/env bash
SCRIPT_VERSION="v0.3.2"

function cout()
{
  echo -e "${1}"
}

function cin()
{
  echo -e "${1}"
}

function cerr()
{
  echo -e "${1}"
}
function cwarn()
{
  echo -e "${1}"
}

# Exit on errors
set -e

MODE=$1
COMPANY_ID=$2

# Check MODE set
if [ -z "$MODE" ]; then
  cerr "MODE variable not defined!"
  exit -1
fi

# Check COMPANY_ID set
if [ -z "$COMPANY_ID" ]; then
  cerr "COMPANY_ID variable not defined!"
  exit -1
fi

# Main
cout "3Laws Robot Diagnostic Module updater (${SCRIPT_VERSION})"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi


if [[ "$MODE" == "PACKAGE" ]]; then
  cout "Package update mode"

  # Check input arguments set
  CURRENT_USER=$3
  if [ -z "$CURRENT_USER" ]; then
    CURRENT_USER=$(whoami)
  fi

  LAWS3_DIR=$4
  if [ -z "$LAWS3_DIR" ]; then
    LAWS3_DIR="${HOME}/3lawsRoboticsInc"
  fi

  SRVNAME=$5
  if [ -z "$SRVNAME" ]; then
    SRVNAME=3laws_rdm_ros2.service
  fi

  # Check if rdm installed
  PACKAGE_DIR="3laws_${COMPANY_ID}"
  if [ ! -d "${LAWS3_DIR}/${PACKAGE_DIR}" ]; then
      cerr "The Diagnostic Module doesn't seem to be installed"
      exit -1
  fi

  (
    cd $LAWS3_DIR/$PACKAGE_DIR

    # Check if git repo has been modified
    CMD1="cd ${LAWS3_DIR}/${PACKAGE_DIR}; git fetch origin &> /dev/null"
    CMD2="cd ${LAWS3_DIR}/${PACKAGE_DIR}; git merge-base --is-ancestor HEAD origin/master || echo 1"
    CMD3="cd ${LAWS3_DIR}/${PACKAGE_DIR}; git status --porcelain"
    if [ -z "${SUDO}" ]; then
      runuser -l "${CURRENT_USER}" -c "${CMD1}"
      DIRTY1=$(runuser -l "${CURRENT_USER}" -c "${CMD2}")
      DIRTY2=$(runuser -l "${CURRENT_USER}" -c "${CMD3}")
    else
      eval $CMD1
      DIRTY1=$(eval $CMD2)
      DIRTY2=$(eval $CMD3)
    fi

    if [[ -n "$DIRTY1" || -n "$DIRTY2" ]]; then
      cerr "Changes detected to Diagnostic Module repo, updated aborted!"
      exit -1
    fi

    # Check if need updating
    CMD="cd ${LAWS3_DIR}/${PACKAGE_DIR}; git diff --quiet HEAD origin/master -- || echo 1"
    if [ -z "${SUDO}" ]; then
      UPDATE_AVAILABLE=$(runuser -l "${CURRENT_USER}" -c "${CMD}")
    else
      UPDATE_AVAILABLE=$(eval $CMD)
    fi

    if [[ -z "$UPDATE_AVAILABLE" ]]; then
      cout "No update available."
      exit 0
    fi
    cout "Update available, updating..."

    # Check if service is running
    SERVICE_ACTIVE=0
    systemctl cat -- ${SRVNAME} &> /dev/null && SERVICE_ACTIVE=1
    if [[ "$SERVICE_ACTIVE" == 1 ]]; then
      SERVICE_ACTIVE=1
      systemctl is-active --quiet ${SRVNAME} || SERVICE_ACTIVE=0
      if [[ "$SERVICE_ACTIVE" == 1 ]]; then
        cout "Stopping daemon..."
        {
          $SUDO systemctl stop ${SRVNAME} &> /dev/null
        } || {
          cwarn "Failed to stop Diagnostic Module service, you may have to restart it by hand!"
        }
      fi
    fi

    # Update
    {
      cout "Pulling repo..."
      CMD="cd ${LAWS3_DIR}/${PACKAGE_DIR}; git pull &> /dev/null"
      if [ -z "${SUDO}" ]; then
        runuser -l "${CURRENT_USER}" -c "${CMD}"
      else
        eval $CMD
      fi
    } || {
      cerr "Failed to update repo!"
      exit -1
    }

    # Restart service
    if [[ "$SERVICE_ACTIVE" == 1 ]]; then
      cout "Restarting daemon..."
      {
        $SUDO systemctl start ${SRVNAME} &> /dev/null
      } || {
        cwarn "Failed to start Diagnostic Module service, you may have to start it by hand!"
      }
    fi
  )
elif [[ "$MODE" == "DOCKER" ]]; then
  cout "Docker update mode"

  DOCKER_CONFIG="--config ${HOME}/.docker/3laws"

  DOCKER_IMAGE_LINK=$3
  if [ -z "$DOCKER_IMAGE_LINK" ]; then
    DOCKER_REGISTRY=ghcr.io/3lawsrobotics
    DOCKER_IMAGE_NAME="3laws_rdm_${COMPANY_ID}"
    DOCKER_IMAGE_LINK="${DOCKER_REGISTRY}/${DOCKER_IMAGE_NAME}:latest"
  fi

  SRVNAME=$4
  if [ -z "$SRVNAME" ]; then
    SRVNAME=3laws_rdm_docker.service
  fi

  # Check if service is running
  SERVICE_ACTIVE=0
  systemctl cat -- ${SRVNAME} &> /dev/null && SERVICE_ACTIVE=1
  if [[ "$SERVICE_ACTIVE" == 1 ]]; then
    SERVICE_ACTIVE=1
    systemctl is-active --quiet ${SRVNAME} || SERVICE_ACTIVE=0
    if [[ "$SERVICE_ACTIVE" == 1 ]]; then
      cout "Stopping daemon..."
      {
        $SUDO systemctl stop ${SRVNAME} &> /dev/null
      } || {
        cwarn "Failed to stop Diagnostic Module service, you may have to restart it by hand!"
      }
    fi
  fi

  # Update
  cout "Pulling docker image..."
  {
    docker ${DOCKER_CONFIG} pull $DOCKER_IMAGE_LINK
  } || {
    cwarn "Failed to pull docker image!"
  }

  # Restart service
  if [[ "$SERVICE_ACTIVE" == 1 ]]; then
    cout "Restarting daemon..."
    {
      $SUDO systemctl start ${SRVNAME} &> /dev/null
    } || {
      cwarn "Failed to start Diagnostic Module service, you may have to start it by hand!"
    }
  fi

else
  cerr "Unkown update mode, should be one of : [PACKAGE, DOCKER]"
  exit -1
fi

cout "Update completed!"
exit 0
