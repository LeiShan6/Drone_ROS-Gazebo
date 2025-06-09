#!/usr/bin/env python
"""
@author: jea2142

Description: Attmept to recreatet position control
"""


import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3Stamped

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from threading import Thread
from tf.transformations import quaternion_from_euler


class offb_test_node:
    
    def __init__(self):
        print("Initializing")        
        self.testHeight = 5;        #Currently hard coded height
        self.attitude_flag = True
        self.threadTest = True
                
        
        self.testClockTiming
        self.setInitVar()
        self.setPubSub(30)          # Currently hard-coded flight rate (Hz)
        #self.wait_for_topics(10)
        self.setPrelimPoints()
        self.setArmTest(True, 5)
    
        
        
        self.changeModeTest("OFFBOARD", 5)
        #self.setOffboard()
        
        #Running actual mission        
#        self.engageBasicflight()
#        self.engageFlightInputs()
        self.setFlightHold()
     
     
     
     
     
        
    def setInitVar(self):
        print("In set Variables method")
        self.state = State()
        self.att = AttitudeTarget()
        self.desPosition = PositionTarget()
        self.desPose = PoseStamped()
        self.actPose = PoseStamped()
        self.thrust = Thrust()
        
        
        
        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }
        
        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            #rospy.loginfo("ROS services are up")
            print("ROS services are up")
        
        except rospy.ROSException:
            self.fail("failed to connect to services")
        
        
    def setPubSub(self, flightFrequency):
        print("Set Publishers and Subscribers")
        rospy.init_node('Sys_ID_Commands', anonymous=True)        

        # Position and acceleration setpoint publisher
        self.positionPub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


        # Attitude and thrust publisher (thrust is published so that
        ### MAVROS will accept attitude setpoints, but is not used)
        self.posePub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.thrustPub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=1)
        
        # Local pose subscriber, to check when vehicle reaches a setpoint
        self.poseSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.getPose)
        
        # State subscriber mainly just to check Offboard status
        self.stateSub = rospy.Subscriber('mavros/state', State, self.getState)


        # Set publish/subscribe rate
        self.rate = rospy.Rate(flightFrequency)  # 50hz
        
        
        # Set mode and arming servers
        self.setArmingSrv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.setModeTestSrv = rospy.ServiceProxy('mavros/set_mode', SetMode)
       
#                       # send setpoints in seperate thread to better prevent failsafe
#        self.att_thread = Thread(target=self.setPrelimPoints, args=())
#        self.att_thread.daemon = True
#        self.att_thread.start()
       
#       
#        rospy.loginfo("Clock test start")
#        now = rospy.get_rostime()
#        rospy.sleep(10)
#        then = rospy.get_rostime()
#        diff = then-now
#        rospy.loginfo("The time difference is"+ str(diff))
       
       
    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def testClockTiming(self):
        rospy.loginfo("Clock test start")
        now = rospy.get_rostime()
        rospy.sleep(10)
        then = rospy.get_rostime()
        diff = then-now
        rospy.loginfo("The time difference is"+ str(diff))
       
       
       
       
    def setPrelimPoints(self):
        self.desPose.pose.position.x = 0
        self.desPose.pose.position.y = 0
        self.desPose.pose.position.z = 2
        
        
        # 6/02 Adding slight yaw direction
#        yaw_degrees = 20  # North
#        yaw = math.radians(yaw_degrees)
#        quaternion = quaternion_from_euler(yaw, 0, 0)
#        self.desPose.pose.orientation = Quaternion(*quaternion)
            
        # 9/20 THIS SECTION NOT NEEDED IN THREADED ATTEMPTS
        if not self.threadTest:
            for i in range(100):
                self.posePub.publish(self.desPose)
                self.rate.sleep
            rospy.loginfo("Finished publishing preliminary setpoints")
        
        else:
            # Threading testing
            self.publishReady = False
            self.pos_thread = Thread(target=self.engageThreadedFlight, args=())
            self.pos_thread.daemon = True
            self.pos_thread.start()
        
    
    
    
    def changeModeTest(self, desMode, timeout):
        print(self.state.mode)
        rospy.wait_for_service('mavros/set_mode')
        rospy.loginfo("setting FCU mode: {0}".format(desMode))
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            print("In set Offboard loop")
            if self.state.mode == desMode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, 5))
                break
            else:
                try:
                    print("In try loop of Offboard case")
                    res = self.setModeTestSrv(0, desMode)  # 0 is custom mode
                    print("Is setMode sending command " + str(res))
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    
    def setOffboard(self):
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/set_mode')
        try:
            base_mode = 0
            custom_mode = "OFFBOARD"
            #derp
            out = change_mode(base_mode, custom_mode)
            if out:
                rospy.loginfo("Successfully changed to offboard mode")
        except rospy.ServiceException:
            rospy.loginfo("Service call failed")


    def setArmTest(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.setArmingSrv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def engageBasicflight(self):
        rospy.loginfo("run simple mission")
        while not rospy.is_shutdown():
            self.posePub.publish(self.desPose)
    
            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
            
    def engageThreadedFlight(self):
        rospy.loginfo("run complex mission")
        self.rate = rospy.Rate(30)  # Hz 
#        if self.publishReady == False:
        while self.publishReady == False:
#            while not rospy.is_shutdown():
            self.posePub.publish(self.desPose)
    
            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
        if self.publishReady == True:
            self.engageFlightInputs()
            
            

    def engageFlightInputs(self):
        rospy.loginfo("publish threaded points")
        self.getFlightxt()
        currentLine = 1
#        self.rate = rospy.Rate(40)
        rospy.loginfo(self.desPose.pose)
#        if (self.state.mode == "OFFBOARD" and self.state.armed == True):
#            print("BOTH MODES PROPERLY SET!!!")
        while not rospy.is_shutdown():
            self.getSetpoint(currentLine)
#            rospy.loginfo(str(self.rate))
            rospy.loginfo(self.desPose.pose)
            self.posePub.publish(self.desPose)
#            self.desPose.pose.position.y += .01
            if currentLine < self.maxlinelen:
                currentLine += 1
            elif currentLine == self.maxlinelen:
                self.publishFinal = True
            self.rate.sleep()

            
    def getFlightxt(self):
        trajfile = open(r'/home/jea2142/catkin_ws/src/flight_inputs/scripts/sysID_flight_input.txt')
        self.lines = trajfile.readlines()
        self.maxlinelen = len(self.lines) - 1 


    def getSetpoint_QUAT(self, currentLine):
               
        
        data = self.lines[currentLine]
        datasplit = data.split(",")
        datasplit[2] = datasplit[2].rstrip()
        datarray = np.array(map(float, datasplit))
        if (self.attitude_flag):
            self.desPose.pose.position.x = datarray[0]
            self.desPose.pose.position.y = datarray[1]
#           self.desPose.pose.position.z = datarray[2]
            self.desPose.pose.orientation.w = datarray[3]
            self.desPose.pose.orientation.x = datarray[4]
            self.desPose.pose.orientation.y = datarray[5]
            self.desPose.pose.orientation.z = datarray[6]
            

    def getSetpoint(self, currentLine):
               
        
        data = self.lines[currentLine]
        datasplit = data.split(",")
        datasplit[2] = datasplit[2].rstrip()
        datarray = np.array(map(float, datasplit))
        if (self.attitude_flag):
#           self.desPose.pose.position.x = datarray[0]
#           self.desPose.pose.position.y = datarray[1]
#           self.desPose.pose.position.z = datarray[2]
        
           self.roll = math.radians(datarray[3])
           self.pitch = math.radians(datarray[4])
           self.yaw = math.radians(datarray[5])
#            self.quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
           self.quaternion = quaternion_from_euler(0, 0, self.yaw)
           self.desPose.pose.orientation = Quaternion(*self.quaternion)

#
#    def getSetpoint(self, currentLine):
#        data = self.lines[currentLine]
#        datasplit = data.split(",")
#        datasplit[2] = datasplit[2].rstrip()
#        datarray = np.array(map(float, datasplit))
#        
#        self.quaternion = datarray[3:7]
#        self.desPose.pose.orientation = Quaternion(*self.quaternion)

    def setFlightHold(self):
        self.reachPosition(0, 0, 2, 0.1)

        print("In hold to switch publishing type "+ str(self.reached))
        self.publishReady = True
        self.publishFinal = False
        holdRate = rospy.Rate(5)
        while not self.publishFinal:
            holdRate.sleep()



    def getState(self, data):
        self.state = data
        
    def getPose(self, data):
        self.actPose = data
#        self.test = self.actPose.pose
#        print("did this work?")
        

        
        
    def reachPosition(self, x, y, z, offset):
        """offset: meters"""
        self.reached = False
        timeout = 10
        loop_freq = 30 #Hz
        positionRate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            desired = np.array((x, y, z))
            pos = np.array((self.actPose.pose.position.x,
                            self.actPose.pose.position.y,
                            self.actPose.pose.position.z))
            if np.linalg.norm(desired - pos) < offset:
                self.reached = True
                print("Reached Hover Point")
                break
            
            
            try:
                positionRate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
        
        
    def log_topic_vars(self):
        """log the state of topic variables. Add July... Eventually update"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")

        
        
        
if __name__ == '__main__':
    print("Python Script has opened") 
    derp = offb_test_node()
    print(derp.testHeight)
    print(derp.state.mode)
#    print(derp.test)
    print("tada")
    