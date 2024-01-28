#!/usr/bin/bash

screen -S emulator -X quit
rm -r port_file.txt || echo "ready to roll"

cur_dir=$(pwd)

screen -S emulator -dm bash -c "echo $pwd;python3 serial_emulator.py -f gps-data.txt; echo $pwd"

sleep 3

port=$(cat port_file.txt)


for ((i=0;i<1;i++))
do
    

    python3 cloner.py $i $1

    cd eece5554/gnss || cd EECE5554/gnss

    rm -rf build
    rm -rf devel

    catkin_make

    source devel/setup.bash

    
    python3 $cur_dir/executer.py $i $port

    cd $cur_dir


done

screen -S emulator -X quit

rm -r port_file.txt