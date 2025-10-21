#!/bin/bash

TARGET_HOST="baidu.com"
export LD_LIBRARY_PATH=/opt/frodobots/lib/
while true; do
    if ping -c 1 -W 1 "$TARGET_HOST" > /dev/null 2>&1; then
        echo "$TARGET_HOST is reachable, starting the application..."
        /opt/frodobots/teleop_agent /opt/frodobots/teleop.ini
        break
    else
        echo "$TARGET_HOST is not reachable, retrying..."
    fi
    sleep 1
done
