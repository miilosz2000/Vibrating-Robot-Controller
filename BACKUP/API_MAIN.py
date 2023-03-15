# -*- coding: utf-8 -*-
"""
API for Coppeliasim 

V1.0 

Milosz Placzek


API Port: 19990

V-REP Folder: C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu
"""

try: 
    import sim # Sim file associated with location of the API
    import random 
    import matplotlib.pyplot as plt
    import time
    import math
    from gym import Env, spaces
    import numpy as np
    import cv2
    import PIL.Image as Image
except: 
    print("SIM loading failed. Please ensure sim.py is located in the same repository")


def NoiseGenerator():
    
    noise_input = random.randrange(10)#generate noise as a random value 0-15
    noise_s = random.randrange(2) #Determine whether noise is positive (Random)
    if(noise_s==0): noise_input=noise_input #Does not invert the noise
    else:noise_input = -noise_input #Invert noise so that it is negative
    
    return noise_input #Return noise value as the function output 

class GraphPlot:
    def __init__(self):
        self.dataX = []#Create data array for x-coordinate
        self.dataY = []#Create data array for y-coordinate
    def AddData(self,dataX,dataY,run): #Function to add data to the array 
        self.run = run 
        self.dataX.append(dataX) #add coordinates to the array
        self.dataY.append(dataY)
        
    def CreatePlot(self):
        runs = len(self.dataX) #find length of data array
        
        for iter in range(runs):
            plt.plot(self.dataX[iter],self.dataY[iter],label="Run: {}".format(iter+1)) #plot all individual plots together 
                
        plt.show()#show ploot 



class API:  #### Class for the API communication, all API events occur within this class
    def __init__(self): 
        
        sim.simxFinish(-1) # End pre-existing connections 
        self.clientID = sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim
        self.episodeReward = []
        if(self.clientID!=1): #Verify that connection was successful
            print("Connection Successful") 
            
            returnCode, pingTime = sim.simxGetPingTime(self.clientID) #Check connection speed
            print("Ping Test Speed: {}".format(pingTime)) 
            
            if(returnCode==0):{print("Connection Status:OK")} #Check connection return ping code, to ensure connection is stable 
            else:{print("Connection Status:FAILURE")}
            
            print("Initialisation Complete - Awaiting Simulation")
            
            sim.simxSetIntegerParameter(self.clientID, sim.sim_intparam_dynamic_engine, 3, sim.simx_opmode_oneshot) #Ensure simulation in Newton Engine 
            
            self.error,self.agent = sim.simxGetObjectHandle(self.clientID, "bristle", sim.simx_opmode_oneshot_wait)#create location point
            self.error1,self.goal = sim.simxGetObjectHandle(self.clientID, "Goal", sim.simx_opmode_oneshot_wait)#create location point
           
           
            self.TotalGraph = GraphPlot()#create a graph item 
        else: 
            print("Connection Failed")
    
    def returnState(self):
        location = sim.simxGetObjectPosition(self.clientID, self.agent, -1, sim.simx_opmode_blocking)
        self.temp_x.append(location[1][0])
        self.temp_y.append(location[1][1])       
        
        return location
        
    def rewardSystem(self,totalReward, previousLocation):
        goalLocation = self.goalLoc
        currentLocation = sim.simxGetObjectPosition(self.clientID, self.agent, -1, sim.simx_opmode_blocking)
        print(goalLocation)
        print(currentLocation)
        currentX = currentLocation[1][0]-goalLocation[1][0]
        currentY = currentLocation[1][1]-goalLocation[1][1]
        
        pastX = previousLocation[1][0]-goalLocation[1][0]
        pastY = previousLocation[1][1]-goalLocation[1][1]
        
        currentDistance = math.sqrt((currentX)**2+(currentY)**2)
        pastDistance = math.sqrt((pastX)**2+(pastY)**2)
        
        ###goalReward
        if(currentDistance>0.2):
            
            totalReward -= 1 ##penalty for not at distance 
            
            ####Reward for total distance
            if(currentDistance<pastDistance):#Closer
                 totalReward += 5
            elif(currentDistance>pastDistance):#Further
                 totalReward -= 6
            else:#No progress
                 totalReward -= 3
            
            ####Reward for X change 
            if(pastX>currentX):
                totalReward += 2##Closer
            else:
                totalReward -= 1
                        
            ####Reward for Y change
            if(pastY>currentY):
                totalReward += 2##Closer
            else:
                totalReward -= 1
        
        else:
            totalReward += 20
                
        return totalReward
        
    def storeRewards():
        pass
    
    
    def RunCycle(self,episodes,iterations):
        
        
        for episode in range(episodes): #for each run
        
            print("Starting Simulation: {}".format(episode+1))
            print("------------RUN {}------------".format(episode+1))
            sim.simxStartSimulation(self.clientID, sim.simx_opmode_blocking)#start simulation
            self.temp_x = []
            self.temp_y = []
            self.goalLoc =  sim.simxGetObjectPosition(self.clientID, self.goal, -1, sim.simx_opmode_blocking)
            totalReward = 0
            
            sim.simxCallScriptFunction(self.clientID, 'bristle', sim.sim_scripttype_customizationscript, "sysCall_init",[] , [], [], bytearray(), sim.simx_opmode_blocking)
            time.sleep(1)
            for i in range(iterations):
                
                print("             {}%".format(int((i/iterations)*100)))
                choice = random.randrange(3)
                noise_input = NoiseGenerator() #Generate value for noise
                
                previousLocation = self.returnState()
                self.action_space(choice,noise_input)
                time.sleep(1)    
                totalReward = self.rewardSystem(totalReward, previousLocation)
                
            sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)#stop simulation
            
            self.TotalGraph.AddData(self.temp_x,self.temp_y,episode)
            self.episodeReward.append(totalReward)
            print("             100%")
            print("Ended Simulation : {}".format(episode+1))
            print("Standby..")
            
            
            time.sleep(4) #Sleep time to allow for simulation to close properly, prevent overload
        self.SecureClose() #Ensure closure 
        print(self.episodeReward)
    def action_space(self,choice,noise):
        
        if(choice==0):###ACTION HALT 
            sim.simxCallScriptFunction(self.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_nil", [], [], [], bytearray(), sim.simx_opmode_blocking)
        elif(choice==1):###ACTION LEFT
            sim.simxCallScriptFunction(self.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_left_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)
        elif(choice==2):###ACTION RIGHT
            sim.simxCallScriptFunction(self.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_right_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)
        else:
            print("False action.")
    
    
        
    def SecureClose(self): #method to securly close down connection (ensures future connection is set up appropriately)
        try: 
            sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)#stop simulation if it hasn't stopped, returns E1 if successful
            print("Connection Ended : E1")
        except: 
            print("Conncection Ended : E2")
        try: 
            sim.simxFinish(-1) # End pre-existing connections, return E1 if successful 
            print("API Closed : E1")
        except:     
            print("API Closed : E2")            
    
        self.TotalGraph.CreatePlot()
        

        


Run = API()
Run.RunCycle(2,20) #Run 10 simulations with 25 iterations each 


