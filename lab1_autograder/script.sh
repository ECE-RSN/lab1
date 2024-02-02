#!/usr/bin/bash

screen -S emulator -X quit
rm -r port_file.txt || echo "ready to roll"

cur_dir=$(pwd)

screen -S emulator -dm bash -c "echo $pwd;python3 serial_emulator.py -f gps-data.txt; echo $pwd"

sleep 3

port=$(cat port_file.txt)


for ((i=0;i<1;i++))
do
        
    python3 $cur_dir/executer.py $1 $port

    cd $cur_dir


done

screen -S emulator -X quit

rm -r port_file.txt