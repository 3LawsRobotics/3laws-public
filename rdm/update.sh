#!/usr/bin/env bash
SCRIPT_VERSION="0.5.0"

# Exit on errors
set -e

# Printing utilities
cout() {
  echo -e "$1"
}
cerr() {
  echo -e "$1"
}
cwarn() {
  echo -e "$1"
}
ctitle() {
  echo -e "$1"
}

run_sudo() {
  if [ -n "$SUDO" ]; then
    runuser -l "$CURRENT_USER" -c "$1"
  else
    eval "$1"
  fi
}

# Define some variables
BRANCH=master
COMPANY_ID=$1
HOME_LOCAL=$2
LAWS3_DIR=$3
CURRENT_USER=$4
USERID=$5
GROUPID=$6
ROBOTID=$7

# Process arguments
if [ "$#" -lt 1 ]; then
  cerr "At least 1 arguments required"
  exit 22
fi

if [ -z "$COMPANY_ID" ]; then
  cerr "First argument cannot be empty!"
  exit 22
fi

if [ -z "$HOME_LOCAL" ]; then
  HOME_LOCAL="$HOME"
fi

if [ -z "$LAWS3_DIR" ]; then
  LAWS3_DIR="$HOME_LOCAL/3lawsRoboticsInc"
fi

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
ctitle "3Laws Robot Diagnostic Module updater (v$SCRIPT_VERSION)"

# Checking if root
SUDO="sudo"
if [[ $(id -u) == 0 ]]; then
  SUDO=""
fi

# Install mode
cout "Package update mode"

# Check input arguments set
SRVNAME=$9
if [ -z "$SRVNAME" ]; then
  SRVNAME=3laws_rdm_ros.service
fi

ROS_DISTRO_LOCAL=${10}
if [ -z "$ROS_DISTRO_LOCAL" ]; then
  ROS_DISTRO_LOCAL=$ROS_DISTRO
fi

# Check if rdm installed
PACKAGE_DIR="3laws_$COMPANY_ID"
if [ ! -d "$LAWS3_DIR/$PACKAGE_DIR" ]; then
  cwarn "The Diagnostic Module doesn't seem to be installed"
  exit 2
fi

# Check if git repo has been modified
cd "$LAWS3_DIR/$PACKAGE_DIR"

ORIGINAL_BRANCH=$(run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git rev-parse --abbrev-ref HEAD")

if [[ $ORIGINAL_BRANCH == "HEAD" ]]; then
  echo "Currently on detached head. Aborting update..."
  exit 131
fi

# Stash uncommited changes
UNCOMMITED_CHANGED=$(run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git status --porcelain")
BRANCHED_OFF_CHANGES=$(run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git merge-base --is-ancestor HEAD origin/$BRANCH || echo 1")
if [[ -n $UNCOMMITED_CHANGED ]]; then
  cout "Uncommited changes detected on repo, will stash changes for you and try to pop them after update..."
  run_sudo "git stash push"
fi

checkout_original() {
  if ! run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git checkout $ORIGINAL_BRANCH"; then
    echo "git checkout of original branch '$ORIGINAL_BRANCH' failed."
    exit 131
  fi
}

pop_stash() {
  if [[ -n $UNCOMMITED_CHANGED ]]; then
    cout "Poping stash"
    if ! run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git stash pop"; then
      echo "Stash pop encountered conflicts"
      exit 131
    fi
  fi
}

# Checkout master
if ! run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git checkout $BRANCH"; then
  echo "Failed to checkout master conflicts. Aborting update..."
  exit 131
fi

# Check if update available
run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git fetch origin &> /dev/null"
UPDATE_AVAILABLE=$(run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git diff --quiet HEAD origin/$BRANCH -- || echo 1")
if [ -z "$UPDATE_AVAILABLE" ]; then
  cout "No update available."
  checkout_original
  pop_stash
  exit 0
fi
cout "Update available, updating..."

# Check if service is running
SERVICE_EXISTS=0
SERVICE_ACTIVE=0
systemctl cat -- "$SRVNAME" &>/dev/null && SERVICE_EXISTS=1
if [[ $SERVICE_EXISTS == 1 ]]; then
  SERVICE_ACTIVE=1
  systemctl is-active --quiet "$SRVNAME" || SERVICE_ACTIVE=0
  if [[ $SERVICE_ACTIVE == 1 ]]; then
    cout "Stopping daemon..."
    {
      $SUDO systemctl stop "$SRVNAME" &>/dev/null
    } || {
      cwarn "Failed to stop Diagnostic Module service, you may have to restart it by hand!"
    }
  fi
fi

# Pull repo
{
  cout "Pulling repo..."
  run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git pull &> /dev/null"
} || {
  cerr "Failed to update repo!"
  checkout_original
  pop_stash
  exit 13
}

# Restart service
if [[ $SERVICE_EXISTS == 1 ]]; then
  cout "Updating daemon..."
  {
    LLL_WS="$LAWS3_DIR/$PACKAGE_DIR"
    sed "s+@LLL_WS@+$LLL_WS+g; s+@ROS_DISTRO@+$ROS_DISTRO_LOCAL+g; s+@USERID@+$USERID+g; s+@GROUPID@+$GROUPID+g; s+@LAWS3_ROBOT_ID@+$ROBOTID+g" "$LLL_WS/packages/$SRVNAME" >"$LAWS3_DIR/$SRVNAME"
    $SUDO mv -f "$LAWS3_DIR/$SRVNAME" "/etc/systemd/system/$SRVNAME" &>/dev/null
    $SUDO systemctl daemon-reload
  } || {
    cwarn "Failed to update Diagnostic Module service, you may have to start it by hand!"
  }
fi

# Restart service
if [[ "$SERVICE_ACTIVE" == 1 ]]; then
  cout "Restarting daemon..."
  {
    $SUDO systemctl restart "$SRVNAME" &>/dev/null
  } || {
    cwarn "Failed to start Diagnostic Module service, you may have to start it by hand!"
  }
fi

# Rebase changes
if [[ -n $BRANCHED_OFF_CHANGES ]]; then
  checkout_original
  cout "Changes not pushed detected on repo, will try to rebase for you..."
  if ! run_sudo "cd \"$LAWS3_DIR/$PACKAGE_DIR\"; git rebase origin/$BRANCH"; then
    echo "Rebase encountered conflicts. Aborting update..."
    run_sudo "\"$LAWS3_DIR/$PACKAGE_DIR\"; git rebase --abort"
    exit 131
  fi
fi
pop_stash

cout "Update completed!"
exit 0
