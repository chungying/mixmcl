#!/bin/bash
# This shell creates a directory and generates a string of date and time as text filenames.
# First, to detect if PACKAGE_NAME/data folder exist
# If it doesn't, create the directory.
# Second, to generate the string.
function help(){
  echo "mkdatadir.sh DIRECTORY NAMESPACE"
  echo "DIRECTORY is a path where is going to store files of parameters and collected data."
  echo "          DIRECTORY is created if it doesn't exist"
  echo "NAMESPACE specifies filenames of the text files."
}

if [[ $# < 2 ]]; then
  echo "please enter a path and a prefix"
  help
  exit 1
elif [[ $# > 2 ]]; then
  #TODO deal with __name:= and __log:=
  echo "[WARN]$ME `$DATE`: too many arguments"
  echo $@
  help
#  exit 1
fi

DATE="date +%F-%H%M%S"
ME=`basename "$0"`
DIRECTORY="$1"
# If no master exist, parameters cannot be set in Parameter Server
rosrun mix_mcl roscheck
if [ $? -eq 1 ]; then
  echo "[INFO]$ME `$DATE`: cannot find ros master. please chekc if roscore is invoked." && exit 1
else
  echo "[INFO]$ME `$DATE`: found the ros master"
fi

# When it is an absolute path
if [[ $DIRECTORY == /* ]]; then
#  echo "[INFO]$ME `$DATE`: \"$DIRECTORY\" is an absolute path."
  DIR=$DIRECTORY
else
# When it is a relative path
#  echo "[INFO]$ME `$DATE`: \"$DIRECTORY\" is a relative path."
  DIR="$PWD/$DIRECTORY"
fi

# If $DIR donsn't exist, create $DIR
if [[ ! -e $DIR ]]; then
  mkdir $DIR
  if [[ $? != 0 ]]; then
    echo "$ME `$DATE`: failed to create \"$DIR\"" && exit 1
  fi
else
  test ! -w $DIR && echo "$ME `$DATE`: cannot create text files in \"$DIR\"" && exit 1
fi

# Set the parameter to Parameter Server
param="$2/fullpath"
value="\"$DIR\""
echo "[INFO]$ME `$DATE`: set parameter \"$param\" to Parameter Server with value $value."
rosparam set $param $value
param="$2/timestamp"
value="\"`$DATE`\""
echo "[INFO]$ME `$DATE`: set parameter \"$param\" to Parameter Server with value $value."
rosparam set $param $value
exit 0
