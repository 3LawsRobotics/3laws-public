#!/usr/bin/env bash
SCRIPT_VERSION="v0.4.1"

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

# Define some variables
BRANCH=master
MODE=$1
COMPANY_ID=$2
HOME_LOCAL=$3
LAWS3_DIR=$4
CURRENT_USER=$5
USERID=$6
GROUPID=$7
ROBOTID=$8

# Check inputs set
if [ -z "$MODE" ]; then
  cerr "MODE variable not defined!"
  exit -1
fi

# Check COMPANY_ID set
if [ -z "$COMPANY_ID" ]; then
  cerr "COMPANY_ID variable not defined!"
  exit -1
fi

if [ -z "$HOME_LOCAL" ]; then
  HOME_LOCAL="${HOME}"
fi

if [ -z "$LAWS3_DIR" ]; then
  LAWS3_DIR="${HOME_LOCAL}/3lawsRoboticsInc"
fi
SCRIPT_DIR="${LAWS3_DIR}/scripts"

if [ -z "$CURRENT_USER" ]; then
  CURRENT_USER=$(whoami)
fi

if [ -z "$USERID" ]; then
  USERID=$(id -u)
fi

if [ -z "$GROUPID" ]; then
  GROUPID=$(id -g)
fi

# Main
cout "3Laws Robot Diagnostic Module updater (${SCRIPT_VERSION})"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi

# Install mode
if [[ "$MODE" == "PACKAGE" ]]; then
  cout "Package update mode"

  # Check input arguments set
  SRVNAME=$9
  if [ -z "$SRVNAME" ]; then
    SRVNAME=3laws_rdm_ros2.service
  fi

  ROS_DISTRO_LOCAL=${10}
  if [ -z "$ROS_DISTRO_LOCAL" ]; then
    ROS_DISTRO_LOCAL=$ROS_DISTRO
  fi


  # Check if rdm installed
  PACKAGE_DIR="3laws_${COMPANY_ID}"
  if [ ! -d "${LAWS3_DIR}/${PACKAGE_DIR}" ]; then
      cerr "The Diagnostic Module doesn't seem to be installed"
      exit -1
  fi

  # Check if git repo has been modified
  cd $LAWS3_DIR/$PACKAGE_DIR
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

  # Check package need updating
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
  SERVICE_EXISTS=0
  SERVICE_ACTIVE=0
  systemctl cat -- ${SRVNAME} &> /dev/null && SERVICE_EXISTS=1
  if [[ "$SERVICE_EXISTS" == 1 ]]; then
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

  # Pull repo
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
  if [[ "$SERVICE_EXISTS" == 1 ]]; then
    cout "Updating daemon..."
    {
      cd $LAWS3_DIR/$PACKAGE_DIR/rdm
      sed "s+@LAWS3_WS@+$(pwd)+g; s+@ROS_DISTRO@+${ROS_DISTRO_LOCAL}+g; s+@USERID@+${USERID}+g; s+@GROUPID@+${GROUPID}+g; s+@LAWS3_ROBOT_ID@+${ROBOTID}+g" ${SRVNAME} \
        > $LAWS3_DIR/${SRVNAME}
      $SUDO mv -f $LAWS3_DIR/${SRVNAME} /etc/systemd/system/${SRVNAME} &> /dev/null
      $SUDO systemctl daemon-reload
    } || {
      cwarn "Failed to update Diagnostic Module service, you may have to start it by hand!"
    }
  fi

  # Restart service
  if [[ "$SERVICE_ACTIVE" == 1 ]]; then
    cout "Restarting daemon..."
    {
      $SUDO systemctl restart ${SRVNAME} &> /dev/null
    } || {
      cwarn "Failed to start Diagnostic Module service, you may have to start it by hand!"
    }
  fi


elif [[ "$MODE" == "DOCKER" ]]; then
  cout "Docker update mode"

  SRVNAME=3laws_rdm_docker
  SRVNAME_FULL=${SRVNAME}.service
  DOCKER_COMPOSE_NAME=${SRVNAME}.docker-compose.yaml
  DOCKER_COMPOSE_PATH=${SCRIPT_DIR}/${DOCKER_COMPOSE_NAME}
  DOCKER_REGISTRY=ghcr.io/3lawsrobotics
  DOCKER_IMAGE_NAME="3laws_rdm_${COMPANY_ID}"
  DOCKER_IMAGE_LINK="${DOCKER_REGISTRY}/${DOCKER_IMAGE_NAME}:latest"
  DOCKER_CONFIG="--config ${HOME_LOCAL}/.docker/3laws"

  # Docker specific input arguments
  NO_PULL=$9

  # Check if service is running
  SERVICE_EXISTS=0
  SERVICE_STOPPED=0
  systemctl cat -- ${SRVNAME_FULL} &> /dev/null && SERVICE_EXISTS=1
  if [[ "$SERVICE_EXISTS" == 1 ]]; then
    SERVICE_ACTIVE=1
    systemctl is-active --quiet ${SRVNAME_FULL} || SERVICE_ACTIVE=0
    if [[ "$SERVICE_ACTIVE" == 1 && -z "${NO_PULL}" ]]; then
      cout "Stopping daemon..."
      SERVICE_STOPPED=1
      {
        $SUDO systemctl stop ${SRVNAME_FULL} &> /dev/null
      } || {
        cwarn "Failed to stop Diagnostic Module service, you may have to restart it by hand!"
      }
    fi
  fi

  # Pull docker image if not NO_PULL
  if [ -z "${NO_PULL}" ]; then
    cout "Pulling docker image..."
    {
      eval docker $DOCKER_CONFIG pull $DOCKER_IMAGE_LINK
    } || {
      cwarn "Failed to pull docker image!"
    }
  fi

  # Check if daemon needs updating
  NEED_COMPOSE_UPDATE=0
  if [[ -f  ${DOCKER_COMPOSE_PATH} ]]; then
    curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/${BRANCH}/rdm/${DOCKER_COMPOSE_NAME}" | \
      sed "s+@DOCKER_IMAGE_LINK@+${DOCKER_IMAGE_LINK}+g; s+@HOME@+${HOME_LOCAL}+g; s+@USERID@+${USERID}+g; s+@GROUPID@+${GROUPID}+g; s+@LAWS3_ROBOT_ID@+${ROBOTID}+g" > ${DOCKER_COMPOSE_PATH}.new
    if [[ -f ${DOCKER_COMPOSE_PATH}.new ]]; then
      if [ -n "$(cat ${DOCKER_COMPOSE_PATH}.new)" ]; then
        cmp -s ${DOCKER_COMPOSE_PATH} ${DOCKER_COMPOSE_PATH}.new || NEED_COMPOSE_UPDATE=1
      else
        rm ${DOCKER_COMPOSE_PATH}.new &> /dev/null
      fi
    fi
  fi

  NEED_SRV_UPDATE=0
  if [[ "$SERVICE_EXISTS" == 1 ]]; then
    curl -fsSL "https://raw.githubusercontent.com/3LawsRobotics/3laws-public/${BRANCH}/rdm/${SRVNAME_FULL}" | \
      sed "s+@DOCKER_COMPOSE_PATH@+${DOCKER_COMPOSE_PATH}+g; s+@USERID@+${USERID}+g; s+@GROUPID@+${GROUPID}+g" > \
      ${SCRIPT_DIR}/${SRVNAME_FULL}.new
    if [[ -f ${SCRIPT_DIR}/${SRVNAME_FULL}.new ]]; then
      if [ -n "$(cat ${SCRIPT_DIR}/${SRVNAME_FULL}.new)" ]; then
        cmp -s /etc/systemd/system/${SRVNAME_FULL} ${SCRIPT_DIR}/${SRVNAME_FULL}.new || NEED_SRV_UPDATE=1
      else
        rm ${SCRIPT_DIR}/${SRVNAME_FULL}.new &> /dev/null
      fi
    fi
  fi

  # Stop service
  if [[ "$NEED_COMPOSE_UPDATE" == 1 || "$NEED_SRV_UPDATE" == 1 ]]; then
    cout "Updating daemon"
    if [[ "$SERVICE_ACTIVE" == 1 && -n "${NO_PULL}" ]]; then
      cout "Stopping daemon..."
      SERVICE_STOPPED=1
      {
        $SUDO systemctl stop ${SRVNAME_FULL} &> /dev/null
      } || {
        cwarn "Failed to stop Diagnostic Module service, you may have to stop it by hand!"
      }
    fi
    if [[ -f  ${DOCKER_COMPOSE_PATH} ]]; then
      {
        if [[ -f ${DOCKER_COMPOSE_PATH}.new ]]; then
          $SUDO mv -f ${DOCKER_COMPOSE_PATH}.new ${DOCKER_COMPOSE_PATH} &> /dev/null
          $SUDO chown ${USERID}:${GROUPID} ${DOCKER_COMPOSE_PATH}
        fi
      } || {
        cwarn "Failed to update docker compose file"
      }
    fi

    if [[ "$SERVICE_EXISTS" == 1 ]]; then
      {
        if [[ -f ${SCRIPT_DIR}/${SRVNAME_FULL}.new ]]; then
          $SUDO mv -f ${SCRIPT_DIR}/${SRVNAME_FULL}.new /etc/systemd/system/${SRVNAME_FULL} &> /dev/null
          $SUDO systemctl daemon-reload
        fi
      } || {
        cwarn "Failed to update daemon file"
      }
    fi
  fi

  # Delete files that may be left behind
  rm ${DOCKER_COMPOSE_PATH}.new &> /dev/null || true
  rm ${SCRIPT_DIR}/${SRVNAME_FULL}.new &> /dev/null || true

  # Restart service
  if [[ "$SERVICE_STOPPED" == 1 ]]; then
    cout "Restarting daemon..."
    {
      $SUDO systemctl restart ${SRVNAME_FULL} &> /dev/null
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
