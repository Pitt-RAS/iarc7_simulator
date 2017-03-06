#!/usr/bin/env bash

lockdir=/tmp/morse_launcher.sh.lock
if mkdir $lockdir 2> /dev/null; then
    trap "rmdir $lockdir" EXIT INT KILL TERM
    cd ../sim
    morse run sim 2> /tmp/morse.log
else
    echo "Failed to create lock directory at /tmp/morse_launcher.sh.lock, exiting..."
fi
