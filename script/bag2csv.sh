#!/bin/bash
# This shell convert a topic in a bagfile into csv file
function help(){
  echo "bag2csv.sh BAG TOPIC"
  echo "BAG filename of a bagfile"
  echo "TOPIC the name of a topic in the bagfile"
}

if [[ $# < 2 ]]; then
  echo "wrong argument number $#"
  help
  exit 1
else
  for (( i = 0 ; i < $1 ; i=$i+1 ));
  do
    roslaunch mixmcl p3dx_2dnav.launch mcl_pkg:=$2 suicide:=true rviz:=false record:=true && echo "finished $i-th " || echo "failed $i-th"
    
  done

fi

