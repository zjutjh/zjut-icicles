#! /bin/bash

###############################################################################
# 1.add Additional startup programs
# start_transbot
# bash /home/pi/Transbot/transbot/start_transbot.sh
# start transbot main program
###############################################################################


###############################################################################
# 2.add to .bashrc last line
# eval "$RUN_TRANSBOT_PROGRAMS"
###############################################################################
source /opt/ros/melodic/setup.bash
source /home/pi/transbot_ws/devel/setup.bash
sleep 3
gnome-terminal &
gnome-terminal -x bash -c 'export RUN_AFTER_BASHRC="python3 /home/pi/Transbot/transbot_main.pyc";exec bash'
exit 0
#gnome-terminal -- bash -c "roscore;exec bash"

#wait
#exit 0
#sleep 5
#source /opt/ros/melodic/setup.bash &
#sleep 3
#roscore &
#sleep 5
#python3 /home/pi/Transbot/transbot_main.py
#exit
