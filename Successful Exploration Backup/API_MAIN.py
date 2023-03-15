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
        plt.legend()
        plt.ylabel("Y")
        plt.xlabel("X \n Rewards Obtained: {}".format(Agent.episodeReward))
        plt.show()#show ploot 


class Agent:
    def __init__(self, spaceInterval, epsilon, alpha, gamma, X_Range, Y_Range):
        self.episodeReward = []
        self.spaceInterval = spaceInterval
        self.EP = epsilon
        self.A = alpha
        self.G = gamma
        number_of_actions = 2
        self.GOAL = 0
        self.lefts = 0
        self.rights = 0
        
        
        self.spaceContainerX = X_Range
        self.spaceContainerY  = Y_Range
        
        spaceX = self.spaceContainerX[1]-self.spaceContainerX[0]
        spaceY = self.spaceContainerY[1]-self.spaceContainerY[0]
        
        #area = spaceX*spaceY
        #boxArea = self.spaceInterval**2
        
        #self.number_of_states = area/boxArea
        self.number_of_states = 3
        self.q_table = np.zeros((int(self.number_of_states), number_of_actions))
        
        
    def trainModel(self):
        pass
        
    
    
    def rewardSystem(self,totalReward, previousLocation,failMark):
        goalLocation = self.goalLoc
        currentLocation = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking)
        #print(previousLocation)
        currentX = currentLocation[1][0]-goalLocation[1][0]
        currentY = currentLocation[1][1]-goalLocation[1][1]
        
        pastX = previousLocation[1][0]-goalLocation[1][0]
        pastY = previousLocation[1][1]-goalLocation[1][1]
        
        currentDistance = math.sqrt((currentX)**2+(currentY)**2)
        pastDistance = math.sqrt((pastX)**2+(pastY)**2)
        actionReward = 0
        rewardProducts = []
        
        ###goalReward
        if(currentDistance>0.2):
            Done = False
            totalReward -= 3 ##penalty for not at distance 
            actionReward -= 3
            
            # try:
            #     if((self.OLD_L_DIS+self.OLD_R_DIS)>(self.L_DIS+self.R_DIS)):
            #         totalReward += 35
            #         actionReward += 35
            #         rewardProducts.append(4)
            #     else:
            #         totalReward -= 15
            #         actionReward -= 15
            #         rewardProducts.append(4)
                    
            # except:
            #     pass
            
            try:
                # if(self.OLD_L_DIS>self.L_DIS or self.OLD_R_DIS>self.R_DIS):
                #     totalReward += 20
                #     actionReward += 20
                #     rewardProducts.append(4)
                # else:
                #     totalReward -= 30
                #     actionReward -= 30
                #     rewardProducts.append(4)
               
                
                if(self.L_DIS>self.R_DIS):
                    distance_ideal = self.R_DIS
                    
                else:
                    distance_ideal = self.L_DIS
                    
                
                ideal_angle = math.acos(1-(0.2**2)/(2*distance_ideal**2))
                actual_angle = math.acos((self.L_DIS**2+self.R_DIS**2-0.2**2)/(2*self.R_DIS*self.L_DIS))
                
               
                
                offset = ideal_angle - actual_angle
                
                if(offset<0.05):
                    totalReward += 20
                    actionReward += 20
                    rewardProducts.append(4)
                elif(0.05<offset<0.1):
                    totalReward += 10
                    actionReward += 10
                    rewardProducts.append(4)
                elif(0.1<offset<0.2):
                     totalReward += 5
                     actionReward += 5
                     rewardProducts.append(4)
                else:
                    totalReward -= 15
                    actionReward -= 15
                    rewardProducts.append(4)
                
                if(self.offset>offset):
                    totalReward += 30
                    actionReward += 30
                    rewardProducts.append(4)
                else:
                    totalReward -= 40
                    actionReward -= 40
                    rewardProducts.append(4)
                    
                self.offset = offset
                
                if(currentDistance<pastDistance):#Closer
                      totalReward += 15
                      actionReward += 15
                      rewardProducts.append(1)
                elif(currentDistance>pastDistance):#Further
                      totalReward -= -30
                      actionReward -= -30
                      rewardProducts.append(2)
                else:#No progress
                      totalReward -= 45
                      actionReward -= 45
                      rewardProducts.append(3)
                
            except:
                print("Exception")
                #print("Ideal Distance: {}".format(distance_ideal))
                try:self.offset = offset
                except:print("Offset Unavailable")
        
        
        
            # try:
            #     if(self.OLD_L_DIS>self.L_DIS):
            #         totalReward += 20
            #         actionReward += 20
            #         rewardProducts.append(4)
            #     else:
            #         totalReward -= 30
            #         actionReward -= 30
            #         rewardProducts.append(4)
                    
            # except:
            #     pass
            # try:
            #     if(self.OLD_R_DIS>self.R_DIS):
            #         totalReward += 20
            #         actionReward += 20
            #         rewardProducts.append(4)
            #     else:
            #         totalReward -= 30
            #         actionReward -= 30
            #         rewardProducts.append(4)
                    
            # except:
            #     pass
            
            # if(currentDistance<pastDistance):#Closer
            #      totalReward += 60
            #      actionReward += 60
            #      rewardProducts.append(1)
            # elif(currentDistance>pastDistance):#Further
            #      totalReward -= 30
            #      actionReward -= 30
            #      rewardProducts.append(2)
            # else:#No progress
            #      totalReward -= 45
            #      actionReward -= 45
            #      rewardProducts.append(3)
            
            ##############ADD if one is bigger, and it goes the other way penaly 
# =============================================================================
#             ####Reward for X change 
#             if(pastX>currentX):
#                 totalReward += 7##Closer
#                 rewardProducts.append(1)
#                 actionReward += 7
#             else:
#                 totalReward -= 10
#                 actionReward -= 10
#                 rewardProducts.append(2)
#                         
#             ####Reward for Y change
#             if(pastY>currentY):
#                 totalReward += 7##Closer
#                 actionReward += 7
#                 rewardProducts.append(1)
#             else:
#                 totalReward -= 10
#                 actionReward -= 10
#                 rewardProducts.append(2)
# =============================================================================
        
        else:
            totalReward += 1000
            actionReward += 1000
            self.GOAL += 1
            Done = True
        
        #print(rewardProducts)
        if(failMark==True):
            totalReward -= 250
            actionReward -= 250
        print("TotalReward: {} ActionReward: {}".format(totalReward, actionReward))
        return totalReward, actionReward, Done
    
   
    def getState(self):
        location = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking)
        self.temp_x.append(location[1][0])
        self.temp_y.append(location[1][1])     
        
        temp = [location[1][0],location[1][1]]
        
        
        state = []
       
         
       
        
        for item in range(2):
            n = 0 #initial state
            lower_bound = -1
            upper_bound = lower_bound + self.spaceInterval
            
            
            
            stateLocation = temp[item]
            #print("LB: {} UB: {} ITEM: {}".format(lower_bound, upper_bound, stateLocation))
            while not (lower_bound <= stateLocation <= upper_bound) and n<100: #n set for a max threshold, prevent timing out 
                n += 1 
                lower_bound += self.spaceInterval
                upper_bound += self.spaceInterval
                #print("LB: {} UB: {} ITEM: {}".format(lower_bound, upper_bound, stateLocation))
            
            if(n<100): 
                locationL = sim.simxGetObjectPosition(API.clientID, API.LR, -1, sim.simx_opmode_blocking)
                locationR = sim.simxGetObjectPosition(API.clientID, API.RR, -1, sim.simx_opmode_blocking)
                
                locL_X = locationL[1][0]
                locL_Y = locationL[1][1]+1
                locR_X = locationR[1][0]
                locR_Y = locationR[1][1]+1
                
                
                try:
                    self.OLD_L_DIS = self.L_DIS
                    self.OLD_R_DIS = self.R_DIS
                except:
                    pass
                
                self.L_DIS = math.sqrt((locL_X)**2+(locL_Y)**2)
                self.R_DIS = math.sqrt((locR_X)**2+(locR_Y)**2)
                
                print(self.L_DIS)
                print(self.R_DIS)                
                if(self.L_DIS==self.R_DIS):
                    state = 2
                    
                elif(self.L_DIS>self.R_DIS):
                    state = 0
                    print("Should turn:RIGHT")
                elif(self.L_DIS<self.R_DIS):
                    state = 1
                    print("Should turn:LEFT")
            else: state.append([-365,-365])
                    
        # #print(state) 
    
        return location, state
  
    def action_space(self,choice,noise):
        
        #if(choice==0):###ACTION HALT 
         #   sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_nil", [], [], [], bytearray(), sim.simx_opmode_blocking)
          #  print("STOP")
        if(choice==0):###ACTION LEFT
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_left_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)
            print("LEFT")
            self.lefts += 1
        elif(choice==1):###ACTION RIGHT
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_right_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)
            print("RIGHT")
            self.rights += 1
        else:
            print("False action.")
            print(choice)
            print(self.q_table)
            
            print(np.argmax(self.q_table[self.state_item]))
            print(self.state_item)
            
      
    def inRangeLocation(self):
        
        location = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking)
        #print(location[1][1])
        
        X = (self.spaceContainerX[0])<location[1][0]<(self.spaceContainerX[1])
        Y = (self.spaceContainerY[0])<location[1][1]<(self.spaceContainerY[1])
        
        if(X and Y): return True
        else: False
    
    def RunCycle(self,episodes,iterations):
        
        
        for episode in range(episodes): #for each run
            #self.EP = (-1/(episode+1))*episode + 1.0
            print("EP: " + str(self.EP))
            print("Starting Simulation: {}".format(episode+1))
            print("------------RUN {}------------".format(episode+1))
            sim.simxStartSimulation(API.clientID, sim.simx_opmode_blocking)#start simulation
            self.temp_x = []
            self.temp_y = []
            self.goalLoc =  sim.simxGetObjectPosition(API.clientID, API.goal, -1, sim.simx_opmode_blocking)
            totalReward = 0
            Done = False
            Fail = False
            #q_table = np.zeros()
            state_item = 0
            
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "sysCall_init",[] , [], [], bytearray(), sim.simx_opmode_blocking)
            time.sleep(1)
            for i in range(iterations):
                
                if(self.inRangeLocation() and not Done):
                    print("             {}%".format(int((i/iterations)*100)))
                    previousLocation, state = self.getState()
                    noise_input = NoiseGenerator() #Generate value for noise
                    #state_item = state[0]*state[1]
                    state_item = state
                   # while not done
                    
                    
                    
                    if random.uniform(0,1) < self.EP:
                        choice = random.randrange(2)
                        
                    else: 
                        try:
                            choice = np.argmax(self.q_table[state_item])
                            
                            
                        except:
                            print("Failure: Argmax(1)")
                            Fail = True
                            totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation, Fail)
                            
                        
                    self.state_item = state_item    
                        
                    
                    
                    self.action_space(choice,noise_input)
                    time.sleep(0.4)    
                    totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation,Fail)
                    
                    newLocation, newState = self.getState()
                    try:
                        newStateMax = np.max(self.q_table[newState]) #numerical value
                        print(state_item)
                        
                        print(choice)
                        self.q_table[state_item,choice] = (1-self.A)*self.q_table[state_item,choice]+self.A*(actionReward + self.G*newStateMax - self.q_table[state_item,choice])
                            
                    except:
                       print("State Failure")
                       #print(np.argmax(self.q_table[state_item]))
                       #print(state_item)
                       
                    
                    #print(newState)
                    #print("Total Reward: {} Action Reward: {}".format(totalReward,actionReward))
                else: 
                    print("Not In Range")
                    break
            
                
            sim.simxStopSimulation(API.clientID, sim.simx_opmode_blocking)#stop simulation
            
            API.TotalGraph.AddData(self.temp_x,self.temp_y,episode)
            self.episodeReward.append(totalReward)
            print("             100%")
            print("Ended Simulation : {}".format(episode+1))
            print("Standby..")
            
            
            time.sleep(4) #Sleep time to allow for simulation to close properly, prevent overload
        
        Done = False 
        
        
        print("FINAL RUN")
        sim.simxStartSimulation(API.clientID, sim.simx_opmode_blocking)#start simulation
        self.temp_x = []
        self.temp_y = []
        self.goalLoc =  sim.simxGetObjectPosition(API.clientID, API.goal, -1, sim.simx_opmode_blocking)
        totalReward = 0
        Done = False
        Fail = False
        #q_table = np.zeros()
        state_item = 0
        
        sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "sysCall_init",[] , [], [], bytearray(), sim.simx_opmode_blocking)
        time.sleep(1)
        for i in range(iterations):
            if(self.inRangeLocation() and not Done):
                print("             {}%".format(int((i/iterations)*100)))
                previousLocation, state = self.getState()
                noise_input = NoiseGenerator() #Generate value for noise
                #state_item = state[0]*state[1]
                state_item = state
               # while not done
                
                
                
                
                choice = np.argmax(self.q_table[state_item])
                        
              
                
                self.action_space(choice,noise_input)
                time.sleep(0.1)    
                totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation,Fail)
                
                newLocation, newState = self.getState()
               
                #print(newState)
                #print("Total Reward: {} Action Reward: {}".format(totalReward,actionReward))
            else: 
                print("Not In Range")
                break
        
            
        sim.simxStopSimulation(API.clientID, sim.simx_opmode_blocking)#stop simulation
        
        API.TotalGraph.AddData(self.temp_x,self.temp_y,episode)
        self.episodeReward.append(totalReward)
        print("             100%")
        print("Ended Simulation : {}".format(episode+1))
        print("Standby..")
        
        
        time.sleep(4) #Sleep time to allow for simulation to close properly, prevent overload
    
        
        API.SecureClose() #Ensure closure 
        print("Total Rewards: {}".format(self.episodeReward))
        self.saveModel()
        print("Total Goals: {} Total Lefts: {} Total Rights: {}".format(self.GOAL, self.lefts, self.rights))
        


    def saveModel(self):
        fileSave = open("model2.txt", "w")
        np.savetxt(fileSave, self.q_table)
        fileSave.close()
        

class API:  #### Class for the API communication, all API events occur within this class
    def __init__(self): 
        
        sim.simxFinish(-1) # End pre-existing connections 
        self.clientID = sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim
        
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
            self.error2,self.LR = sim.simxGetObjectHandle(self.clientID, "LeftReference", sim.simx_opmode_oneshot_wait)#create location point
            self.error3,self.RR = sim.simxGetObjectHandle(self.clientID, "RightReference", sim.simx_opmode_oneshot_wait)#create location point
           
            self.TotalGraph = GraphPlot()#create a graph item 
        else: 
            print("Connection Failed")

    
        
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

EPISODES = 10
ITERATIONS = 50
SPACE_INTERVAL = 0.1
X = [-1.5, 1.5]
Y = [-1.5, 1.5]


EPSILON = 0.2
ALPHA = 0.7
GAMMA = 0.85



API = API()
Agent = Agent(SPACE_INTERVAL, EPSILON, ALPHA, GAMMA, X, Y)
Agent.RunCycle(EPISODES,ITERATIONS) #Run 10 simulations with 25 iterations each 


