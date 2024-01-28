# Requirements

1) You must have screen

        sudo apt-get install screen

2) Driver must be able to launch using

        roslaunch gps_driver driver.launch port:=<any given port>

# Running

1) Kill any pre-existing terminals with your ROS node and your emulator

2) Run

        bash script.sh <your git repository link>


# Debugging

1) If the script has been printing that it is waiting for a topic for more than 10 seconds, it means your launch has failed.

2) Using the    `screen -ls` command in a new terminal, check if you have a screen session called *ros_node* (if you do proceed to step 3). If there is not any session depicts that your node crashes the instant it is launched. To understand the issue, please break the script, run the emulator separately and then launch your node using

        roslaunch gps_driver driver.launch port:=<emulator's port>

3) If you see a screen session named *ros_node* then you can look at the place where your node is failing by using the command  `screen -R ros_node`. If nothing is being printed, it may be because of your node not having any log or print statements. Otherwise, the screen would show why your node is failing. 

    Regardless, you should check if your node echos on topic /gps if you launch it with the emulator and the command

        roslaunch gps_driver driver.launch port:=<emulator's port>



