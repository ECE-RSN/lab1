#!/usr/bin/bash

screen -S emulator -X quit
screen -S ros_node -X quit
rm -f port_file.txt || echo "ready to roll"

cur_dir=$(pwd)

# FIX: cd to autograder directory inside screen session
screen -S emulator -dm bash -c "cd $cur_dir && python3 serial_emulator.py -f gps-data.txt"

sleep 3

port=$(cat port_file.txt)

for ((i=0;i<1;i++))
do
    python3 $cur_dir/executer.py $1 $port
    cd $cur_dir
done

screen -S emulator -X quit
screen -S ros_node -X quit

rm -f port_file.txt