#!/bin/bash
# This shell play a rosbag file and kill all other nodes when the rosbag ends.


function help(){
  echo "kill.sh NODENAME"
  echo "NODENAME is the name of node whose pulse is checked by this shell "
}

if [[ $# < 1 ]]; then
  echo "wrong argument number $#"
  help
  exit 1
else
  #TODO if $1 exist
  echo "checking rosmaster"
  rosrun mixmcl roscheck || exit 3
  echo "checking $1 pulse"
  rosnode ping $1 > /dev/null
  #echo "palying the rosbag file: $1"
  #if [[ "$2" == "mixmcl" ]]; then
  #  echo "delay rosplay for waiting mixmcl"
  #  rosbag play --delay=3 $1 
  #else
  #  rosbag play $1 
  #  echo "palyed the rosbag file: $1"
  #fi
  #EXITCODE=4
fi

#because rosnode kill -a would fail, using "kill -INT -tpgid" will be more efficient
rosnode list && echo "Killed all ros nodes."
source `rospack find mixmcl`/script/kroslaunches.sh && exit 0 || exit 1

