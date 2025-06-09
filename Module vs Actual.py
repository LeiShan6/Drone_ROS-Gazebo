# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 17:14:33 2020

@author: jea2142
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
#import scipy.spatial

from scipy.spatial.transform import Rotation as R
from JA_SysID_Git import JA_SysID


file_name = 'multi08_output.csv'

with open('/home/jea2142/catkin_ws/src/flight_inputs/scripts/Bag_Files/'+file_name) as csv_file:
    csv_reader = csv.reader(csv_file)
    line_count = 0
    time = []
    pose = []               #This array includes just the position
    orientation =[]         #This contains Euler angles after transform
    test_length = []        #This is the quat-Eul-quat values
    quat_og = []            #This is the original quaternion values from bag files


    for row in csv_reader:
        if line_count == 0:
            print('OPENING ROW')
            
        else:
            t_i = float(row[0])
            
            position = [float(row[4]), float(row[5]), float(row[6]) ]
            quaternion = [float(row[7]), float(row[8]), float(row[9]), float(row[10]) ]
            
            if quaternion == [0, 0, 0, 0]:
                euler = [0,0,0]
                marker = line_count
            else:
                rot = R.from_quat(quaternion)
                euler = rot.as_euler('zyx', degrees=True)
            
            # Test section for accuracy of data
#            r = R.from_euler('zyx', euler, degrees=True)
#            quat_test = r.as_quat()
#            test_length.append(quat_test)
#            quat_og.append(quaternion)

            orientation.append(euler)
            pose.append(position)
            time.append(t_i)

            
        line_count += 1
        
    time = np.array(time)      
    pose = np.array(pose)
    orientation = np.array(orientation)
    
    test_length = np.array(test_length)
    quat_og = np.array(quat_og)

    print(pose.shape)
    print(orientation)
    

time = time - time[0]
time = time*(10**-9)

plt.figure(0)
plt.plot(time, pose[:,2])

plt.figure(1)
plt.plot(time, orientation[:,2])

plt.figure(2)
plt.title('Pitch, Yaxis')
plt.plot(time, orientation[:,1])


plt.figure(3)
plt.figure(figsize=(8,6))
plt.title('Yaw, Zaxis')
plt.xlabel('Time (s)')
plt.ylabel('Degrees')
plt.plot(time, orientation[:,0])


plt.figure(4)
#plt.plot(time, test_length[:,2]-quat_length[:,2])
#plt.plot(time,test_length[:,2])

##########################################################################
flight = JA_SysID(30)
    


#    plt.title('Multisine_'+ str(flight.MultiDesign))
#    plt.xlabel('Time Steps')
#    plt.ylabel('Degrees')

testTime = time
testOrientation = orientation[:,0]

#testTime = time[marker+1:-1]
#testTime = testTime - testTime[0]
#testOrientation = orientation[marker+1:-1,0]
testLength = len(flight.finalInput)


derp = np.linspace(0, testLength/30, testLength)
plt.plot(derp, flight.finalInput, testTime, testOrientation[:,0])
plt.title('Designed Input vs Published Input')
#plt.figure(5)

#plt.plot(derp[450:750], flight.finalInput[450:750], derp[450:750], testOrientation[450:750])
#plt.title('Designed Input vs Published Input')
#plt.figure(6)

####### GRAPH OF DIFFERENCES NOT CURRENTLY AVAILABLE
#if testLength < len(testOrientation):
#    testDifference = flight.finalInput - testOrientation[0:len(flight.finalInput)]
#    
#else:
##    testDifference = flight.finalInput[0:len(testOrientation)] - testOrientation
##    testDifference = flight.finalInput[1:301] - testOrientation[0:300]
#    testDifference = flight.finalInput[451:751] 
#    
##plt.plot(testOrientation)
#plt.plot(testDifference)
#plt.title('Input Error over Time')


