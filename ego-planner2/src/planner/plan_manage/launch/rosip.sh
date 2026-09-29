#!/bin/bash
hostip=`hostname -I | awk -F " "  '{print $2}'`
if [[ ! -z $hostip ]]; then
  #hostip=192.168.3.68
  echo HOST_IP:$hostip
  export ROS_IP=$hostip
fi
