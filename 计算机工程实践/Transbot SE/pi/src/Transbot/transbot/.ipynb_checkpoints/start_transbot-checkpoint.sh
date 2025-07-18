#! /bin/bash

###############################################################################
# 1.add Additional startup programs
# start_transbot
# bash /home/jetson/Transbot/transbot/start_transbot.sh
# start transbot main program
###############################################################################


###############################################################################
# 2.add to .bashrc last line
# eval "$RUN_TRANSBOT_PROGRAMS"
###############################################################################

gnome-terminal -- bash -c "cd /root/Transbot/transbot;sleep 3;export RUN_TRANSBOT_PROGRAMS='python3 /root/Transbot/transbot/transbot_main.pyc';exec bash"

wait
exit 0
