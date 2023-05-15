#!/usr/bin/env bash

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

SCRIPT_VERSION="v0.3.0"
MODE=$1
CURRENT_USER=$2
LAWS3_DIR=$3
PACKAGE_DIR=$4
SRVNAME=$5

# Check input arguments set
if [ -z "$MODE" ]; then
  cerr "MODE variable not defined!"
  exit -1
fi
if [ -z "$CURRENT_USER" ]; then
  cerr "CURRENT_USER variable not defined!"
  exit -1
fi
if [ -z "$LAWS3_DIR" ]; then
  cerr "LAWS3_DIR variable not defined!"
  exit -1
fi
if [ -z "$PACKAGE_DIR" ]; then
  cerr "PACKAGE_DIR variable not defined!"
  exit -1
fi
if [ -z "$SRVNAME" ]; then
  cerr "SRVNAME variable not defined!"
  exit -1
fi

# Main
cout "3Laws Robot Diagnostic Module updater (${SCRIPT_VERSION})"

# Checking if root
SUDO="sudo"
if [[ "$(id -u)" == 0 ]]; then
  SUDO=""
fi

(
  cd $LAWS3_DIR

  if [[ "$MODE" == "PACKAGE" ]]; then
    cout "Package update mode"

    # Check if rdm installed
    if [ ! -d "${PACKAGE_DIR}" ]; then
        cerr "The Diagnostic Module doesn't seem to be installed"
        exit -1
    fi

    (
      cd $PACKAGE_DIR
      # Check if git repo has been modified
      runuser -l "${CURRENT_USER}" -c "cd ${LAWS3_DIR}/${PACKAGE_DIR}; git fetch origin &> /dev/null"
      DIRTY1=$(runuser -l "${CURRENT_USER}" -c "cd ${LAWS3_DIR}/${PACKAGE_DIR}; git merge-base --is-ancestor HEAD origin/master || echo 1")
      DIRTY2=$(runuser -l "${CURRENT_USER}" -c "cd ${LAWS3_DIR}/${PACKAGE_DIR}; git status --porcelain")
      if [[ -n "$DIRTY1" || -n "$DIRTY2" ]]; then
        cerr "Changes detected to Diagnostic Module repo, updated aborted!"
        exit -1
      fi

      # Check if need updating
      UPDATE_AVAILABLE=$(runuser -l "${CURRENT_USER}" -c "cd ${LAWS3_DIR}/${PACKAGE_DIR}; git diff --quiet HEAD origin/master -- || echo 1")
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
        runuser -l "${CURRENT_USER}" -c "cd ${LAWS3_DIR}/${PACKAGE_DIR}; git pull &> /dev/null"
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
  else
    cout "Docker update mode"

  fi
)
