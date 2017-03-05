#!/usr/bin/env bash

PROCESS="blender"
RESULT=`pgrep ${PROCESS}`

if [ "${RESULT:-null}" = null ]; then
    cd ../sim
    morse run sim 2> /tmp/morse.log
fi
