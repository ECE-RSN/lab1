import os
import pandas as pd
import time
import rospy
import sys


DATA = "BREAD"
argv=sys.argv
port = str(argv[2])
callback_received = False

def callback(data):

    global DATA
    global callback_received
    print("callback occured")
    callback_received = True
    #if DATA == "None":
    DATA = data
      # Set the flag to True when a callback occurs
    sub.unregister()
    


if __name__ == "__main__":

    home_dir = os.getcwd()
    home_files = os.listdir(os.getcwd())
    #global DATA

    print(home_files)

    assert "src" in  home_files, "No src folder found"

    os.chdir(os.getcwd()+"/src")

    files = os.listdir(os.getcwd())

    assert ( "gps_driver" not in files or "gps-driver" not in files )==True, "ROS Package naming is not correct"

    if ("gps_driver" in files):
        package = "gps_driver"
    else:
        package = "gps-driver"

    
    os.chdir(os.getcwd()+"/"+package)

    assert("msg" not in os.listdir(os.getcwd()+"/msg") and "launch" not in os.listdir(os.getcwd()+"/launch") and "python" not in os.listdir(os.getcwd()+"/python"))==True, "Incorrect folder names, do you have 'python' 'msg' and 'launch' folders?"

    assert("Customgps.msg" not in (os.getcwd()+"/msg"))==True, "No Customgps.msg file found or is the naming convention correct ?"

    assert("standalone_driver.py" not in (os.getcwd()+"/python")), "No standalone_driver.py file found in python folder"

    assert("standalone_driver.launch" not in (os.getcwd()+"/launch"))== True, "No standalone_driver.launch file found in launch folder"

    os.chdir(home_dir)



    os.system("catkin_make")

    os.system('screen -S ros_node -dm roslaunch "'+package+'" standalone_driver.launch port:="'+port+'"')

    print("Screen Running, your ROS node should start within 10 seconds.")

    time.sleep(10)

    rospy.init_node('Evaluator', anonymous=True)
    

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


    
    sub =rospy.Subscriber("/gps", Customgps, callback)    
    cur_time = time.time()
try :
    while "BREAD" in DATA and time.time()-cur_time<30:

        time.sleep(0.5)
        print("waiting for topic")
        print("callback_received :",callback_received)
        print("Received",DATA)
        
        
        
        

    if "BREAD" in DATA:
        comment,penalty = "not publishing over topic gps. Either node never initialized or topic name was incorrect",20

        #df['score'][i] =  str(20-penalty)
        #df['comment'][i] = comment

        #os.chdir(home_dir)

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

try :

    if DATA.header.frame_id.upper() != "GPS1_FRAME":
        cumulative_penalty = cumulative_penalty + 0.2
        cumulative_comment = cumulative_comment + "Frame ID is incorrect. "

    if (DATA.header.stamp.secs%86400)!=2*3600+34*60+58:
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


try:
    if DATA.hdop ==1.0:
        do_nothing= True
    else:
        cumulative_comment,cumulative_penalty = cumulative_comment +"HDOP Parsing incorrect. ",cumulative_penalty + 0.2
        
except:
    cumulative_comment,cumulative_penalty = cumulative_comment +"HDOP Name in message not created correctly. ",cumulative_penalty + 0.2
    msg_structure_ok = False


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
print("Time Stamp for Seconds : ",2*3600+34*60+58)   # hh*3600 + mm*60 + ss

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



print("\n\n\n")
print("TOTAL SCORE OBTAINED : ")
print(str(50-cumulative_penalty*10))
print(" ")
print("COMMENT : ")
print(cumulative_comment)




sys.exit(0)
#return " Driver works ", cumulative_penalty
