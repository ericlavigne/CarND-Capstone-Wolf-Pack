#!/bin/bash

if [ $# -ne 2 ]; then
	echo usage: $0 watch_dir command_to_run
	exit
fi

PROJECT_DIR=$1
COMMAND=$2

while true
do
    echo watching \"$PROJECT_DIR\", file modifications will trigger \"$COMMAND\"
    #inotifywait -r $PROJECT_DIR
    inotifywait -r -e close_write $PROJECT_DIR
    $COMMAND
done