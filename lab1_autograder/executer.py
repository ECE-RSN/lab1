import os
import pandas as pd
import time
import rospy
import sys
from datetime import datetime, timezone
import subprocess


DATA = "BREAD"
argv=sys.argv
port = str(argv[2])
callback_received = False

def callback(data):

    global DATA
    global callback_received
    print("callback occured")
    callback_received = True
    print("Data",data)
    #if DATA == "None":
    DATA = data
      # Set the flag to True when a callback occurs
    # sub.unregister()
    


if __name__ == "__main__":

    # workspace_dir = os.getcwd()
    workspace_dir = str(argv[1])
    print(workspace_dir)
    home_files = os.listdir(workspace_dir)
    #global DATA

    print(home_files)

    assert "src" in  home_files, "No src folder found"

    # src_dir=os.chdir(workspace_dir+"/src")
    src_dir = os.path.join(workspace_dir, "src/")
    # print("sr address",src_dir)

    files = os.listdir(src_dir)
    print(files)

    assert ( "gps_driver" not in files or "gps-driver" not in files )==True, "ROS Package naming is not correct"

    if ("gps_driver" in files):
        package = "gps_driver"
    else:
        package = "gps-driver"

    package_dir=os.path.join(workspace_dir+"src/"+package+"/")

    # print("pkg address",package_dir)

    # msg_dir=os.path.join(workspace_dir+"src/"+package+"/msg")


    # pkg_files = os.listdir(package_dir)
    # print(pkg_files)

    # msg_files = os.listdir(msg_dir)
    # print("msg iles",msg_files)

    assert("msg" in os.listdir(package_dir) and "launch" in os.listdir(package_dir) and "python" in os.listdir(package_dir)), "Required folders 'python', 'msg', and 'launch' are missing."

    assert("Customgps.msg" in os.listdir(package_dir+"msg/")), "No Customgps.msg file found or is the naming convention correct ?"

    assert("standalone_driver.py" in os.listdir(package_dir+"python/")), "No standalone_driver.py file found in python folder"

    assert("standalone_driver.launch" in os.listdir(package_dir+"launch/")), "No standalone_driver.launch file found in launch folder"

    os.chdir(workspace_dir)



    os.system("catkin_make")

    # os.system("source "+workspace_dir+"devel/setup.bash")
    command = "source " + workspace_dir + "devel/setup.bash"
    subprocess.run(["bash", "-c", command])


    os.system('screen -S ros_node -dm roslaunch "'+package+'" standalone_driver.launch port:="'+port+'"')

    print("Screen Running, your ROS node should start within 10 seconds.")

    time.sleep(10)

    

    try :
        from gps_driver.msg import Customgps
    except:
        try:
            from gpsdriver.msg import Customgps
        except:
            try:
                from lab1.msg import Customgps

            except:
                try :
                    from LAB1.msg import Customgps

                except :
                    assert False, "unable to import Customgps.msg, have you sourced devel/setup.bash in the terminal?"


    rospy.init_node('Evaluator', anonymous=True)



    sub =rospy.Subscriber("/gps", Customgps, callback)    
    cur_time = time.time()

    try :
        while "BREAD" in DATA and time.time()-cur_time<30:

            time.sleep(0.5)
            print("waiting for topic")
            # print("callback_received :",callback_received)
            # print("Received",DATA)
            
            
            
            

        if "BREAD" in DATA:
            comment,penalty = "not publishing over topic gps. Either node never initialized or topic name was incorrect",20

            #df['score'][i] =  str(20-penalty)
            #df['comment'][i] = comment

            #os.chdir(workspace_dir)

            #df.to_csv(file,index = False)
            
            print(comment)

            sys.exit(0)
            sys.exit(0)

    except:
        pass



    print("Received Message on Topic")

    cumulative_penalty = 0
    cumulative_comment = " "
    msg_structure_ok = True


    time_ = int(datetime.combine(datetime.now(timezone.utc).date(), datetime.strptime("02:34:58", "%H:%M:%S").time(), timezone.utc).timestamp())


    try :

        if DATA.header.frame_id.upper() != "GPS1_FRAME":
            cumulative_penalty = cumulative_penalty + 0.2
            cumulative_comment = cumulative_comment + "Frame ID is incorrect. "

        if (DATA.header.stamp.secs)!=time_:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "seconds conversion is incorrect. "
            
        if DATA.header.stamp.nsecs!=0.23*10**9:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "nano-second calculation is incorrect. "

    except:

        cumulative_comment,cumulative_penalty = cumulative_comment +"header Name in message not created correctly. ",cumulative_penalty + 1
        msg_structure_ok = False

    try:
        if abs(DATA.latitude - 34.02019816666667) > 0.00001:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "Latitude calculation incorrect. "
    except:

        cumulative_comment,cumulative_penalty = cumulative_comment +"Latitude Name in message not created correctly. ",cumulative_penalty +0.5
        msg_structure_ok = False

    try:
        if abs(DATA.longitude + 118.41129950000001) > 0.00001:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "Longitude calculation incorrect. "
    except:

        cumulative_comment,cumulative_penalty = cumulative_comment +"Longitude Name in message not created correctly. ",cumulative_penalty +0.5
        msg_structure_ok = False

    try:
        if abs(DATA.utm_easting - 369695.4373543182) > 1:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "UTM Easting is incorrect. "
    except:

        cumulative_comment,cumulative_penalty = cumulative_comment +"UTM Easting Name in message not created correctly. ",cumulative_penalty + 0.5
        msg_structure_ok = False

    try:
        if abs(DATA.utm_northing - 3765293.4953880184) > 1:
            cumulative_penalty = cumulative_penalty + 0.5
            cumulative_comment = cumulative_comment + "UTM Northing is incorrect. "
    except:

        cumulative_comment,cumulative_penalty = cumulative_comment +"UTM Northing Name in message not created correctly. ",cumulative_penalty + 0.5
        msg_structure_ok = False

    try:
        if abs(DATA.letter != "S"):
            cumulative_penalty = cumulative_penalty + 0.2
            cumulative_comment = cumulative_comment + "Letter is incorrect. "

    except:
        cumulative_comment,cumulative_penalty = cumulative_comment +"Letter Name in message not created correctly. ",cumulative_penalty + 0.2
        msg_structure_ok = False

    try:
        if DATA.zone !=11:
            cumulative_penalty = cumulative_penalty + 0.2
            cumulative_comment = cumulative_comment + "Zone is incorrect. "
            
    except:
        cumulative_comment,cumulative_penalty = cumulative_comment +"Zone Name in message not created correctly. ",cumulative_penalty + 0.2
        msg_structure_ok = False

    try:
        if DATA.altitude ==0.0:
            do_nothing= True
        else:
            cumulative_comment,cumulative_penalty = cumulative_comment +"Altitude Parsing incorrect. ",cumulative_penalty + 0.2
            
    except:
        cumulative_comment,cumulative_penalty = cumulative_comment +"Altitude Name in message not created correctly. ",cumulative_penalty + 0.2
        msg_structure_ok = False

    #########################################################################################################
    # Uncomment below lines to use UTC
    # try:
    #     if DATA.UTC !=None:
    #         do_nothing= True
    #     else:
    #         cumulative_comment,cumulative_penalty = cumulative_comment +"UTC Parsing incorrect. ",cumulative_penalty + 0.2
            
    # except:
    #     cumulative_comment,cumulative_penalty = cumulative_comment +"UTC Name in message not created correctly. ",cumulative_penalty + 0.2
    #     msg_structure_ok = False


    # try:
    #     if DATA.hdop ==1.0:
    #         do_nothing= True
    #     else:
    #         cumulative_comment,cumulative_penalty = cumulative_comment +"HDOP Parsing incorrect. ",cumulative_penalty + 0.2
            
    # except:
    #     cumulative_comment,cumulative_penalty = cumulative_comment +"HDOP Name in message not created correctly. ",cumulative_penalty + 0.2
    #     msg_structure_ok = False


    if msg_structure_ok==False:
        msg_structure_ok = False
        cumulative_comment,cumulative_penalty = cumulative_comment +"ROS message structure is incorrect. ",cumulative_penalty + 1

    if callback_received:
                print("callback received")
                os.kill(os.getpid(), 2)


    rospy.spin()

    os.system("screen -S ros_node -X quit")


    os.system("clear")



    print("Received output from student: \n\n")
    print(DATA)



    print("\n\n")
    print("Correct Output: ")
    print("Time Stamp for Seconds : ",time_)   # hh*3600 + mm*60 + ss

    print("Time Stamp for Nano-Seconds : ",0.23*10**9) # .ss * 10^9

    print("latitude : ",34.02019816666667)  # DD+mm.mm/60 , multiply by -1 if there is S

    print("longitude : ", -118.41129950000001) #DDD+mm.mm/60 , multiply by -1 if there is W

    print("easting : ",369695.4373543182)

    print("northing : ",3765293.4953880184)

    print("zone : ",11)

    print("letter : ","S")

    print("header : ","GPS1_Frame")

    print ("hdop: 1.0")

    print ("altitude: 0")



    # print("\n\n\n")
    # print("TOTAL SCORE OBTAINED : ")
    # print(str(50-cumulative_penalty*10))
    # print(" ")
    # print("COMMENT : ")
    # print(cumulative_comment)




    sys.exit(0)
    #return " Driver works ", cumulative_penalty
