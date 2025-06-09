#!/usr/bin/env python


import rospy
import numpy as np
import time
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3Stamped

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

omnipose = None
quad = None
linecount = 0
acc_rad = 0.1
sp = PositionTarget()
sa = PoseStamped()
omerr = PoseStamped()
tr = Thrust()
tr.thrust = 1.0 # Dummy value to allow attitudes to be published

def traj_pub(trajfile, length):
    global linecount
    # Load position, acceleration, and attitude setpoint from traj file
    sp, sa = getSetpoint(trajfile, linecount)

    # Initialize ROS node (for publishing/subscribing)
    rospy.init_node('omni_trajectory', anonymous=True)

    ### Trajectory Publishers ###
    # Position and acceleration setpoint publisher
    pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Position error publisher (for recording to bagfiles)
    puberr = rospy.Publisher('omnipos/error', PoseStamped, queue_size=1)

    # Attitude and thrust publisher (thrust is published so that
    # MAVROS will accept attitude setpoints, but is not used)
    pub2 = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=1)
    pubt = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=1)

    # Local pose subscriber, to check when vehicle reaches a setpoint
    sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback)
    
    stateSub = rospy.Subscriber('mavros/state', State, getState)

    # Set publish/subscribe rate
    rate = rospy.Rate(30)  # 50hz


    # Publish preliminary setpoints before arming to enable PX4 OFFBOARD mode
    for k in range(50):
        pub.publish(sp)
        pub2.publish(sa)
        pubt.publish(tr)
        rate.sleep()
    rospy.loginfo("Finished publishing preliminary setpoints")

    # Arm the vehicle and switch to OFFBOARD mode
    print("Attempting to arm 1")    
    setArm(True)
    rospy.loginfo('Armed')
    setOffboard()
    time.sleep(1)

    while not rospy.is_shutdown():
        # Publish setpoints
        pub.publish(sp)
        pub2.publish(sa)
        pubt.publish(tr)

        # Get current position, setpoint position, and error betwen them
        #print("Attempting to send positions")omnipose.state =
        x = omnipose.position.x
        y = omnipose.position.y
        z = omnipose.position.z
        spx = sp.position.x
        spy = sp.position.y
        spz = sp.position.z
        errx = x - spx
        erry = y - spy
        errz = z - spz

        # Save errors to position error message and then publish
        omerr.pose.position.x = errx
        omerr.pose.position.y = erry
        omerr.pose.position.z = errz
        puberr.publish(omerr)

        # Debug outputs
        #rospy.loginfo('Traj file length: %s', length)
        #rospy.loginfo('X error: %s m', errx)
        #rospy.loginfo('Y error: %s m', erry)
        #rospy.loginfo('Z error: %s m', errz)

        # Check if vehicle has reached a setpoint to within the acceptance radius
        # and get the next setpoint if so. If the vehicle reaches the end of the
        # trajectory file, repeat from the beginning, ignoring the first setpoint
        if ((abs(errx) < acc_rad) and (abs(erry) < acc_rad) and (abs(errz) < acc_rad)):
            if (linecount == length):
                linecount = 1
            else:
                linecount = linecount+1
            rospy.loginfo('Arrived at setpoint')
            sp,sa = getSetpoint(trajfile, linecount)

        # Sleep for the amount of time specified in the rate command above
        rate.sleep()



def setArm(armed):
    print("In Arming Method")    
    rospy.wait_for_service('mavros/cmd/arming')
    print("Past service")
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        armService(armed)
    except rospy.ServiceException:
        rospy.loginfo("Service arming call failed")
    print(quad.armed)        
        

def setOffboard():
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rospy.wait_for_service('/mavros/set_mode')
    try:
        base_mode = 0
        custom_mode = "OFFBOARD"
        out = change_mode(base_mode, custom_mode)
        if out:
            rospy.loginfo("Successfully changed to offboard mode")
    except rospy.ServiceException:
        rospy.loginfo("Service call failed")

def callback(data):
    global omnipose
    omnipose = data.pose
    
def getState(data):
    global quad
    quad = data


def getSetpoint(trajfile, linecount):
    data = trajfile[linecount]
    datasplit = data.split(",")
    datasplit[2] = datasplit[2].rstrip()
    datarray = np.array(map(float, datasplit))
    sp.position.x = datarray[0]
    sp.position.y = datarray[1]
    sp.position.z = datarray[2]
    if (attitude_flag):
        sa.pose.orientation.w = datarray[3]
        sa.pose.orientation.x = datarray[4]
        sa.pose.orientation.y = datarray[5]
        sa.pose.orientation.z = datarray[6]
    if (len(datarray) > 7):
        sp.acceleration_or_force.x = datarray[7]
        sp.acceleration_or_force.y = datarray[8]
        sp.acceleration_or_force.z = datarray[9]

    return sp, sa


if __name__ == '__main__':
    print("Python Script has opened")    
    attitude_flag = True
    testfile = open(r'/home/jea2142/catkin_ws/src/flight_inputs/scripts/traj_circle.txt')
    lines = testfile.readlines()
    linelen = len(lines) - 1
    try:
        traj_pub(lines, linelen)
    except rospy.ROSInterruptException:
        pass
