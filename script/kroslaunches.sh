#!/bin/bash
# This shell kill all the processes related to roslaunch

STR=`ps x O tpgid | grep roslaunch`
arr=()
IFS=$'\n' read -d '' -r -a arr <<< "$STR"
for (( i = 0 ; i < ${#arr[@]} ; i=$i+1 ));
do
  line=${arr[${i}]}
  echo "dealing with the line: $line"
  gpid="initial"
  count=0
  for word in $line
  do
    gpid=$word
    if [[ $count == 1 ]]; then
      break
    fi
    count=$((count+1))
  done
  
  echo "killing the group $gpid"
  kill -INT -$gpid
done

