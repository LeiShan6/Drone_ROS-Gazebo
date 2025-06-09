# -*- coding: utf-8 -*-
"""
Spyder Editor

This file currently contains frequency sweep flight inputs

@title: ASDFSDAFSDAFSDAFSDAFSDAFASD 
@author: Johnee Angarita
@year: 2020-Feb-21

@description: This is the integration file from John's dissertation. 
""" 
 
import csv
import numpy as np
import math
from scipy.signal import chirp
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


class JA_SysID():

    def __init__(self, flightFrequency):
        # State 0: Position (x, y , z, Roll, Pitch, Yaw)
        # State 1: Yawrate
    
        # Case 0: Chirp
        # Case 1: HMPIM
        # Case 2: Multisine
        # Case 3: Combine the three systems singular test (Work on later...)
    
        # Axis for Position
        # [0, 1, 2] = [X, Y, Z]
        # [3, 4, 5] = [Roll, Pitch, Yaw]
        # Axis for Velocity
        # [0, 1, 2, 3] = [dX, dY, dZ, dYaw]
    
        stateNum = 0
        caseNum = 2
        axisNum = 5
        repNum = 2

        self.f_i = 0.10                      #Initial frequency
        self.f_f = 3                       #Final Frequency
    
    
        self.setSysIDParam(stateNum ,caseNum, repNum, axisNum, flightFrequency)
        self.setFlightInput()
        self.setVelCommandsForm(flightFrequency)
#        self.setVelCommandsFormQuat()

           
        
        
    def setSysIDParam(self, stateNum ,caseNum, repNum, axisNum, flightFrequency):
        self.caseNum = caseNum
        self.repNum = repNum
        self.axis = axisNum
        self.flightFrequency = flightFrequency
        self.commandRate = flightFrequency

        
        # State 0: Position (x, y , z, Roll, Pitch, Yaw)
        # State 1: Yawrate
        if (stateNum == 0):
            self.posTest = True
            self.velTest = False
            self.inputComLength = 6
            
        elif (stateNum ==1):
            self.posTest = False
            self.velTest = True
            self.inputComLength = 4
            
        else:
            print("Improper State Setting")
            
    
        
        
            
            
    def setFlightInput(self):
        # Define which case of input excitaion to use
         # Case 0: Chirp
         # Case 1: HMPIM
         # Case 2: Multisine
         # Case 3: Combine the three systems singular test (Work on later...)
         
            # Then number of repetitions
        if self.caseNum == 0 :
            self.setChirp()

                            
        elif self.caseNum == 1:
            self.setHMPIM()
          
        elif self.caseNum == 2:
            self.setMultisine()
         

#        elif self.caseNum == 3:
#            self.intInput = np.append( self.u_HMPIM_dot, self.du2, axis=0)
#            self.finalInput = np.append(self.f_sweep, self.intInput, axis=0)
#            
#            for x in range(0,self.repNum):
#                self.finalInput = np.append(self.f_sweep, self.intInput, axis=0)
                
        else: print("Improper case")
        
        self.finalInput = self.prelimInput
        for x in range(0,self.repNum):
            self.finalInput = np.append(self.finalInput, self.prelimInput, axis=0)
        
    
    
            
    def setChirp(self):
        if self.posTest == True:
        
            # Test Parameters
            self.totalTestT= 1000;
            self.commandRate = self.flightFrequency;
    
            #####################################################################
            # FREQUENCY SWEEP parameters (scaleChirp to adjust)
            # (i.e.) Frequencies, length and scale of movements
            self.delta_chirp = 25
            self.delta_trim  = 5
            self.scaleChirp  = 1.0
    
    
            # Define ending/final time of movements
            self.t_chirp1 = self.delta_chirp
            self.t_trim1  = self.t_chirp1+self.delta_trim
    
    
            #Create time arrays of movments to submit into chirp
            self.tc1 = np.linspace(0, self.t_chirp1, self.delta_chirp*self.commandRate)
            self.tt1 = np.linspace(self.t_chirp1, self.t_trim1, self.delta_trim*self.commandRate)
    
    
            #Remove time series overlap
            self.tt1 = self.tt1[1:]
    
    
    
            # chirp takes in frequencies as Hertz
            self.w1 = self.scaleChirp*chirp(self.tc1, f0=self.f_i, f1=self.f_f, t1=self.t_chirp1, method='linear', phi=-90)
           
    
            # Setting zero value array for length of prescribed vector
            self.z1 = np.zeros(len(self.tt1))
    
    
            self.t_sweep = np.concatenate((self.tc1, self.tt1), axis=0)
            self.f_sweep = np.concatenate((self.w1, self.z1), axis=0)
            self.noise = np.random.normal(0,0.5,len(self.f_sweep))
            self.f_sweep = self.f_sweep+self.noise
            
            # Key line to be returned
            self.prelimInput = self.f_sweep
            
        else:
            self.setChirpParam()
            
            # Note f_chirp could be used in place of f_sweep...            
            self.X_chirp = np.sin(self.Sigma1)/self.Sigma2
            self.X_chirp_dot = (self.b + self.a*self.t_Hebert) * np.cos(self.Sigma1)/self.Sigma2
            self.u_Chirp_dot = np.append(self.X_chirp_dot, self.u_Htrim, axis=0)
            
            #Key line to be returned
            self.prelimInput = self.u_Chirp_dot
            
                
            
    def setChirpParam(self):
        # Create time segments
        self.delta_HMPIM = 10 #Total chirp time in sec
        self.t_Hebert = np.linspace(0, self.delta_HMPIM, self.delta_HMPIM*self.commandRate)
        self.delta_htrim = 5
        self.t_mt = np.linspace(self.delta_HMPIM, self.delta_HMPIM+self.delta_htrim, self.delta_htrim*self.commandRate)

        # Trim zeros for velocity commands
        self.u_Htrim = np.zeros(len(self.t_mt))

        # Defining parameters
        self.a = 2*math.pi*(self.f_f-self.f_i)/self.delta_HMPIM
        self.b = 2*math.pi*self.f_i
        self.n = 1.5 #extinction factor
        self.G = 2.5 #HMPIM Gain (i.e the scale factor)

        self.Sigma1 = 0.5*self.a*self.t_Hebert**2 +self.b*self.t_Hebert
        self.Sigma2 = self.b+0.5*self.a
        self.Sigma2ext = self.b+0.5*self.a*self.t_Hebert**self.n
        
        self.minFactor = self.Sigma2ext/self.Sigma2
                
        self.HMPIMplot = False
        if self.HMPIMplot == True:
            plt.figure()
            plt.title('HMPIM_n='+ str(self.n))
            plt.xlabel('Time Steps')
            plt.ylabel('Amp Minimize Value')
            #plt.plot(flight.Sigma2ext, flight.t_Hebert, label='first plot')
            #plt.plot(flight.Sigma2, flight.t_Hebert, label='second plot')
            #plt.hlines(flight.Sigma2, 0, 10)
            plt.plot(self.t_Hebert, self.minFactor)
       
       
       
    def setHMPIM(self):
        # New section from Hebert (HMPIM), scale factor to adjust G
        self.setChirpParam()
        
        self.X_HMPIM = self.G*np.sin(self.Sigma1)/self.Sigma2ext
        self.X_HMPIM_dot = self.G*((self.b+self.a*self.t_Hebert)*np.cos(self.Sigma1)/self.Sigma2ext - self.a*self.n*self.t_Hebert**(self.n-1)*np.sin(self.Sigma1)/2/(self.Sigma2ext**2))

        self.u_HMPIM = np.append(self.X_HMPIM, self.u_Htrim, axis=0)
        self.u_HMPIM_dot = np.append(self.X_HMPIM_dot, self.u_Htrim, axis=0)
        
        if self.posTest == True:
            self.prelimInput = self.u_HMPIM
        
        else:
            self.prelimInput = self.u_HMPIM_dot
            
            

    def setMultisine(self):
        # UPDATE JULY20: Not most efficient implementation but works...
        # UPDATE NOV16: Cases beyond original Alalbsi 4 axis example
    
        
        
        # Test Cases for Omega/Phase Choice
        # Case 0: Alabsi original 4 axis case
        # Case 1: GA optimized 3 axis for 2Hz
        # Case 2: GA optimized 3 axis for 3Hz
        
        # MultiDesign: Which Omega/Phase axis to choose
        # [1,2,3] for RPY or [1,2,3,4] for four rotors
        
        ## MultiInput: For multiaxis testing 
        self.MultiCase = 1
        self.MultiDesign = 3
        self.MultiInput = False   
        
        ### PART 1: Initial Parameters and specification 
        self.delta_mtrim = 5
        self.A = 2.5
        self.u1  = 0
        self.du1 = 0

        ## PART 2: Import proper Omega/Phase case based on specification
        # Use phase in rad/s and Omega in Hz

        if self.MultiCase == 0: #Alabsi
            self.delta_multi = 10
            self.dF = 0.1
            
            
            self.Omega1 = np.array([0.2, 0.6, 1.0, 1.4, 1.8, 2.2, 2.6])
            self.Omega2 =  np.array([x+self.dF for x in self.Omega1])
            self.Omega3 =  np.array([x+self.dF for x in self.Omega2])
            self.Omega4 =  np.array([x+self.dF for x in self.Omega3])
            self.Omega = np.array([self.Omega1, self.Omega2, self.Omega3, self.Omega4] )
            
            self.Phase1 = np.array([90.0, 261.9, 69.5, 60.0, 224.6, 229.7, 200.3])
            self.Phase2 = np.array([92.5, 268.5, 70.2, 61.1, 227.2, 221.7, 204.6])
            self.Phase3 = np.array([111.9, 301.6, 118.3, 141.1, 324.9, 328.1, 331.2])
            self.Phase4 = np.array([187.7, 76.8, 318.3, 28.8, 271.1, 340.4, 42.7])
            
            self.Phase1 = math.pi*self.Phase1/180
            self.Phase2 = math.pi*self.Phase2/180
            self.Phase3 = math.pi*self.Phase3/180
            self.Phase4 = math.pi*self.Phase4/180
            self.Phase = np.array([self.Phase1, self.Phase2, self.Phase3, self.Phase4])
            
            
        if self.MultiCase == 1: # rpy2Hz
            self.delta_multi = 20
            self.dF = 0.05
            
            self.Omega1 = np.array([0.1, 0.25, 0.4, 0.55, 0.7, 0.85, 1, 1.15, 1.3, 1.45, 1.6, 1.75, 1.9])
            self.Omega2 =  np.array([x+self.dF for x in self.Omega1])
            self.Omega3 =  np.array([x+self.dF for x in self.Omega2])
            self.Omega = np.array([self.Omega1, self.Omega2, self.Omega3] )
            
            self.Phase1 = np.array([153, 38, 294, 319, 219, 71, 190, 135, 172, 250, 92, 176, 238])
            self.Phase2 = np.array([245, 359, 29, 31, 291, 109, 127, 321, 31, 68, 350, 217, 312])
            self.Phase3 = np.array([142, 314, 150, 299, 128, 114, 270, 235, 185, 228, 239, 87, 87])
            
            self.Phase1 = math.pi*self.Phase1/180
            self.Phase2 = math.pi*self.Phase2/180
            self.Phase3 = math.pi*self.Phase3/180
            self.Phase = np.array([self.Phase1, self.Phase2, self.Phase3])
            
            
            
            
        if self.MultiCase == 2: # rpy3Hz
            self.delta_multi = 20
            self.dF = 0.1
            
            self.Omega1 = np.array([0.1, 0.4, 0.7 , 1, 1.3, 1.6, 1.9, 2.2, 2.5, 2.8])
            self.Omega2 =  np.array([x+0.1 for x in self.Omega1])
            self.Omega3 =  np.array([x+0.1 for x in self.Omega2])
            self.Omega = np.array([self.Omega1, self.Omega2, self.Omega3] )
            
            self.Phase1 = np.array([282, 345, 47, 138, 68, 315, 113, 336, 7, 261])
            self.Phase2 = np.array([280, 337, 37, 68, 242, 105, 65, 323, 43, 281])
            self.Phase3 = np.array([130, 180, 128, 289, 254, 257, 127, 168, 262, 106])
        
            self.Phase1 = math.pi*self.Phase1/180
            self.Phase2 = math.pi*self.Phase2/180
            self.Phase3 = math.pi*self.Phase3/180
            self.Phase = np.array([self.Phase1, self.Phase2, self.Phase3])
        
        ## Part 3: Create time and input vectors
        self.t_m = np.linspace(0, self.delta_multi, self.delta_multi*self.commandRate)
        self.t_mt = np.linspace(self.delta_multi, self.delta_multi+self.delta_mtrim, self.delta_mtrim*self.commandRate)

        # Trim zeros for velocity commands
        self.du_trim = np.zeros(len(self.t_mt))
        
        
        # Forge the input u or du
        if self.MultiInput == False:
            i = self.MultiDesign - 1
            for x in range(0, len(self.Omega1)):
                self.u1 = self.u1+self.A*np.sin(self.Omega[i][x]*2*math.pi*self.t_m + self.Phase[i][x])
                self.du1 = self.du1+(self.A*2*math.pi*self.Omega[i][x])*np.cos(self.Omega[i][x]*2*math.pi*self.t_m + self.Phase[i][x])
                    
        elif self.MultiInput == True:
            print("Not yet implemented")
                   
                
                
        # Now add trim value
        self.u1=np.append(self.u1, self.du_trim, axis=0)
        self.du1=np.append(self.du1, self.du_trim, axis=0)

        #### Additional code for reading MATLAB phase outputs
#        PhaseImport = np.loadtxt(fname = "C:/Users/johna/Documents/MATLAB/multisineOptimalPhase.txt")
#        print(PhaseImport)
#        print(len(PhaseImport))
        
     
        
        
        # Key return Value
        if self.posTest == True:
            self.prelimInput = self.u1
        
        else:
            self.prelimInput = self.du1    
    
    
    
    
    
    # Now doing quaternion change within the ROS NODE  
    def setVelCommandsForm_DEPRECIATED(self):
        # Put in form of [velx, vely, velz, ang] for pos
        # Put in form [x, y, z, q1, q2, q3, q4] for vel
    
        # Set up zeros to dictate which axis to excite
        
        self.inputCommands = np.zeros((len(self.finalInput), self.inputComLength))
        for x in range(0,len(self.finalInput)):
            self.inputCommands[x][self.axis] = self.finalInput[x]
            
            if self.posTest == True:
                self.euler = self.inputCommands[x][3:6]
                self.rot = R.from_euler('xyz', self.euler, degrees=True)
                self.quaternion = self.rot.as_quat()
                self.inputCommands[x][3:7] = self.quaternion
                
                
                
                
    def setVelCommandsForm(self,flightFrequency):
        # Put in form of [velx, vely, velz, ang] for pos
        # Put in form [x, y, z, q1, q2, q3, q4] for vel
    
        # Set up zeros to dictate which axis to excite
        
        self.inputCommands = np.zeros((len(self.finalInput), self.inputComLength))
        self.testTime = len(self.finalInput)/flightFrequency
        for x in range(0,len(self.finalInput)):
            self.inputCommands[x][self.axis] = self.finalInput[x]
            
            if self.posTest == True:
                self.euler = self.inputCommands[x][3:6]
                self.inputCommands[x][3:6] = self.euler
                
    
    def setVelCommandsFormQuat(self):
        quatTest = True
        
        self.quatCommands = np.zeros((len(self.finalInput), self.inputComLength+1))
        
        for x in range(0, len(self.finalInput)):
            roll = self.inputCommands[x][3]
            pitch = self.inputCommands[x][4]
            yaw = self.inputCommands[x][5]
            
            rot = R.from_euler('zyx', [roll, pitch, yaw], degrees = True)
            quatTest = rot.as_quat()
            self.quatCommands[x][3:7] = quatTest
            
        self.inputCommands = self.quatCommands
    
    
    def setTxtFile(self, fileName):
        file = open(fileName,'w')
        Xmax = len(self.inputCommands[0])
        Xmax = Xmax -1
        
        for (x,y) in np.ndenumerate(self.inputCommands):
            file.write(str(y))
            if (x[1] == Xmax):
                file.write('\n')
            else:
                file.write(',')

        file.close()




        
    def getAmountOfSteps(self):
        return len(self.inputCommands)
        
    
    def getVelVector4(self, timeStep):
        return self.inputCommands[timeStep]




## Last method just for confiming some input quarternion design changes
    def getTxtFile_DEPRECIATED(self):
        with open('/home/jea2142/catkin_ws/src/flight_inputs/scripts/sysID_flight_input.txt') as txt_file:
            txt_reader = csv.reader(txt_file)
            self.orientation =[]
            
            
            for row in txt_reader:
                quaternion = [float(row[3]), float(row[4]), float(row[5]), float(row[6]) ]
            
                rot = R.from_quat(quaternion)
                euler = rot.as_euler('zxy', degrees=True)
                self.orientation.append(euler)
                
            self.orientation = np.array(self.orientation)
            
        plt.figure(1)
        plt.title('Roll, Xaxis')
        plt.plot(self.orientation[:,0])

        plt.figure(2)
        plt.title('Pitch, Yaxis')
        plt.plot(self.orientation[:,1])

        plt.figure(3)
        plt.title('Yaw, Zaxis')
        plt.plot(self.orientation[:,2])




if __name__ == '__main__':
    print("Python Script has opened") 
    flight = JA_SysID(30)
    derp = np.linspace(0, len(flight.finalInput)/30, len(flight.finalInput))
    

    plt.figure(0)
    plt.figure(figsize=(8,6))
#    plt.title('Multisine_'+ str(flight.MultiDesign))
#    plt.xlabel('Time Steps')
#    plt.ylabel('Degrees')
    plt.plot(derp, flight.finalInput)

    print(flight.testTime)
    

    testVariable =  flight.finalInput    
    
    flight.setTxtFile('sysID_flight_input.txt')


    

