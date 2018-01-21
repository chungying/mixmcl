#!/bin/bash
# This shell repeats p3dx_2dnav.launch for N times
function help(){
  echo "rp.sh N MCL_PKG"
  echo "N is the number of repeating."
  echo "MCL_PKG is the name of MCL algorithms, amcl, mixmcl, and mcmcl."
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

