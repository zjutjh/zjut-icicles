#! /bin/bash

len1=` ps -ef|grep transbot_main.pyc |grep -v grep| wc -l`
echo "Number of processes="$len1

if [ $len1 -eq 0 ] 
then
    echo "transbot_main.pyc is not running "
else
    # ps -ef| grep transbot_main.py| grep -v grep| awk '{print $2}'| xargs kill -9  
    camera_pid=` ps -ef| grep transbot_main.pyc| grep -v grep| awk '{print $2}'`
    kill -9 $camera_pid
    echo "transbot_main.pyc killed, PID:"
    echo $camera_pid
fi
sleep .1
