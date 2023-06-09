# -*- coding: utf-8 -*-
"""
API for Coppeliasim 

V3.1

Capabilities:
    -Train the model to simple orientation Q-Table that can determine how to rotate itself towards the goal
    -Run the model using the Q-Table,and an integrated interface with Coppeliasim

Milosz Placzek
University of Sheffield
For the purpose of Master Degree in Aerospace Engineering Final Dissertation


API Port: 19990 

V-REP Folder: C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu


"""


try: 
    import sim # Sim file associated with location of the API
    import random 
    import matplotlib.pyplot as plt
    import time
    import math
    import numpy as np
    import sys 
except: 
    print("SIM loading failed. Please ensure sim.py is located in the same repository")

def NoiseGenerator(): ##Generates noise that is sent through API to the simulation 
    
    noise_input = random.randrange(10)#generate noise as a random value 0-15
    noise_s = random.randrange(2) #Determine whether noise is positive (Random)
    
    if(noise_s==0): noise_input=noise_input #Does not invert the noise
    else:noise_input = -noise_input #Invert noise so that it is negative
    
    return noise_input #Return noise value as the function output 

class GraphPlot: ##Generates plot for the final results, every episode is graphed 
    def __init__(self):
        self.dataX = []#Create data array for x-coordinate
        self.dataY = []#Create data array for y-coordinate
    def AddData(self,dataX,dataY,run): #Function to add data to the array 
        self.run = run #Determine which run it is associated with
        self.dataX.append(dataX) #add coordinates to the array
        self.dataY.append(dataY) 
        
    def CreatePlot(self):
        runs = len(self.dataX) #find length of data array
        
        for iter in range(runs): #go through every run
            plt.plot(self.dataX[iter],self.dataY[iter],label="Run: {}".format(iter+1)) #plot all individual plots together 
        plt.legend()
        plt.ylabel("Y")
        plt.xlabel("X \n Rewards Obtained: {}".format(Agent.episodeReward))
        plt.show()#show plot 


class Agent: #Agent/Controller for the simulation
    def __init__(self, spaceInterval, epsilon, alpha, gamma, X_Range, Y_Range):
        self.episodeReward = [] #an array for the total reward for an entire episode 
        self.spaceInterval = spaceInterval #interval that the simulation is divided into (in m)
        self.EP = epsilon #proportion of random actions during learning
        self.A = alpha #how quickly the algorithm learns
        self.G = gamma #how much the algorithm depends on long-term reward
        number_of_actions = 2 #how many actions the robot is capable of making 
        self.GOAL = 0 #how many times the goal has been reached (should increase when training)
        self.lefts = 0 #how many left actions are executed
        self.rights = 0 #how many right actions are executed
        self.closest_session = 10000 #closest distance to goal, used to initate the value, and the value is recreated with every episode
        
        self.spaceContainerX = X_Range #array for allowable space in the x range
        self.spaceContainerY  = Y_Range #array for allowable space in the y range
        
        #### code may be obselete
        #spaceX = self.spaceContainerX[1]-self.spaceContainerX[0] #overall range size for x
        #spaceY = self.spaceContainerY[1]-self.spaceContainerY[0] #overall range size for y 
        
        self.number_of_states = 3 #possible states of the robot, 2 is virtually not achievable as it would require perfect alignment. Implemeneted for robustness
        self.q_table = np.zeros((int(self.number_of_states), number_of_actions)) #creates a blank Q-Table for training session 
        
    
    def rewardSystem(self,totalReward, previousLocation,failMark): #reward calculation for each step 
        goalLocation = self.goalLoc #location of goal in the simulation 
        currentLocation = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking) #use api to find current agent location 
        
        currentX = currentLocation[1][0]-goalLocation[1][0] #current x location of agent
        currentY = currentLocation[1][1]-goalLocation[1][1] #current y location of agent
        
        ####code removed for current version, may return for future version so not obselete
        #pastX = previousLocation[1][0]-goalLocation[1][0] #previous x location of agent
        #pastY = previousLocation[1][1]-goalLocation[1][1] #previous y location of agent
        #pastDistance = math.sqrt((pastX)**2+(pastY)**2) #previous total distance between goal and agent
        

        currentDistance = math.sqrt((currentX)**2+(currentY)**2)
        
        pastDistance = math.sqrt((pastX)**2+(pastY)**2)
        actionReward = 0
        rewardProducts = []

        
        actionReward = 0 #preset reward for action 
        
        
        
        if(currentDistance>0.2): #is agent less than 0.2m from goal, if not then proceed 
            Done = False #not within 0.2m
            totalReward -= 3 #penalty for not at distance 
            actionReward -= 3 
            
            
            try:
               
                if(self.L_DIS>self.R_DIS): #determine which is smaller to find optimal angle 
                    distance_ideal = self.R_DIS #set as closest distance, allow for ideal angle calculation 
                    
                else:
                    distance_ideal = self.L_DIS
                
                cc = 0.175**2 + (abs(currentDistance-0.1))**2
                
                maxPossible = math.sqrt(cc) #maximum value for L to be so that it's below 90 degrees and not in false orientation
                
                ######## Add threshold angle, change the rest to check angle. If smaller than threshold, and a false image, invert the view 
                ideal_angle = math.acos(1-(0.2**2)/(2*distance_ideal**2)) #ideal angle, determined by closest possible distance as if the goal is ahead 
                
                
                var = (self.L_DIS**2+self.R_DIS**2-0.2**2)/(2*self.R_DIS*self.L_DIS) #find actual angle, where largest angle would be the ideal angle. Anything below that is sub optimal
                
                #####investigate further 
                if(var>1):#avoids rounding error that leads to beyond 1 
                    var = 1
                else: pass

                

                actual_angle = math.acos(var)
                
               
                if(distance_ideal<maxPossible):#is it the other way around
                    pass
                else:
                    actual_angle = actual_angle-math.pi
                offset = ideal_angle - actual_angle
                #offset = offset + math.pi/2
                
                if(0<offset<0.005):

                    totalReward += 20
                    actionReward += 20
                    
                elif(0.005<offset<0.01):
                    totalReward += 10
                    actionReward += 10
                   
                elif(0.01<offset<0.02):
                     totalReward += 5
                     actionReward += 5
                     
                else:
                    totalReward -= 15 #penalty 
                    actionReward -= 15
                    
                

                # if(currentDistance<self.closest_session):#Closer
                #       totalReward += 20
                #       actionReward += 20
                #       self.closest_session = currentDistance
                #       rewardProducts.append(1)
                      
                # elif(currentDistance>self.closest_session):#Further
                #       totalReward -= -45
                #       actionReward -= -45
                #       rewardProducts.append(2)
                # else:#No progress
                #       totalReward -= 45
                #       actionReward -= 45
                #       rewardProducts.append(3)
                
                
                if(self.offset>offset):

                    totalReward += 45 #30
                    actionReward += 45 #30
                    
                else:
                    totalReward -= 40
                    actionReward -= 40

                self.offset = offset #refresh global offset
                
                
            except:
                print("Exception")
                #####should provide more info 
                
                try: # may fail if the first iteration to occur
                    self.offset = offset
                    
                except:
                    print("Offset Unavailable::Should be first iteration only")
                    
                    

        else:
            totalReward += 2000 #Maximise long term award, if the goal is reached
            actionReward += 2000
            self.GOAL += 1
            Done = True
        
        if(failMark==True): #Penalise for leaving simulation space 
            totalReward -= 250
            actionReward -= 250
        
        return totalReward, actionReward, Done
    
   
    def getState(self):
        location = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking) #get current location 
        self.temp_x.append(location[1][0])#temporary location x
        self.temp_y.append(location[1][1])#temporary location y 
        
        temp = [location[1][0],location[1][1]] #create array for space 
        

        for item in range(2): #range of states 
            n = 0 #initial state
            lower_bound = self.spaceContainerY[0] #lower bound of space 
            upper_bound = lower_bound + self.spaceInterval #upper bound of space 
            
            
            
            stateLocation = temp[item] #checks both x and y variable 
            
            while not (lower_bound <= stateLocation <= upper_bound) and n<100: #n set for a max threshold, prevent timing out 
                n += 1 #goes to next state 
                lower_bound += self.spaceInterval #updates boundaries of current state, as each state has different boundaries
                upper_bound += self.spaceInterval
            
            if(n<100): 
                locationL = sim.simxGetObjectPosition(API.clientID, API.LR, -1, sim.simx_opmode_blocking)#get distance to left node
                locationR = sim.simxGetObjectPosition(API.clientID, API.RR, -1, sim.simx_opmode_blocking)#get distance to right node 
                
                goalLocation = self.goalLoc #get location of goal 
                
                locL_X = abs(goalLocation[1][0]-locationL[1][0]) #find distance for x and y for both variables (refer to thesis for extra information)
                locL_Y = abs(goalLocation[1][1]-locationL[1][1])
                locR_X = abs(goalLocation[1][0]-locationR[1][0])
                locR_Y = abs(goalLocation[1][1]-locationR[1][1])
                
                ####may be obselete
                # try:
                #     self.OLD_L_DIS = self.L_DIS #assign current values for future reference 
                #     self.OLD_R_DIS = self.R_DIS
                # except:
                #     pass
                
                self.L_DIS = abs(math.sqrt((locL_X)**2+(locL_Y)**2)) #find actual distance to goal, determine orientation 
                self.R_DIS = abs(math.sqrt((locR_X)**2+(locR_Y)**2))

                if(self.L_DIS==self.R_DIS):#set state depending on goal location to robot. this state is virtually impossible, but used for robustness
                    state = 2
                    
                elif(self.L_DIS>self.R_DIS):
                    state = 0
                    
                elif(self.L_DIS<self.R_DIS):
                    state = 1
                    
            else: state = 10000 #if out of bound, will crash episode and start next 

    
        return location, state
  
    def action_space(self,choice,noise):

        if(choice==0):###ACTION LEFT
        
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_left_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)  
            self.lefts += 1  #add to left actions in the training episodes
        elif(choice==1):###ACTION RIGHT
        
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_right_50", [noise], [], [], bytearray(), sim.simx_opmode_blocking)
            self.rights += 1
        else:
            ####if incorrect action selected, should never occur 
            print("False action.")
            print(choice)
            print(self.q_table)
            print(np.argmax(self.q_table[self.state_item]))
            print(self.state_item)
            sys.exit()
            
      
    def inRangeLocation(self): #Is the agent within the defined space? 
        
        location = sim.simxGetObjectPosition(API.clientID, API.agent, -1, sim.simx_opmode_blocking) #find location of agent

        
        X = (self.spaceContainerX[0])<location[1][0]<(self.spaceContainerX[1]) #is it within defined space ?
        Y = (self.spaceContainerY[0])<location[1][1]<(self.spaceContainerY[1])
        
        if(X and Y): return True #output depending if true or false, should be within limits 
        else: False
    
    def RunCycle(self,episodes,iterations):
        
        
        for episode in range(episodes): #for each run of training 
            self.closest_session = 100000 #set a closest_session variable that is the closest point to the goal, this is then compared vs actual and updated
            print("EP: " + str(self.EP))
            print("Starting Simulation: {}".format(episode+1))
            print("------------RUN {}------------".format(episode+1))
            
            sim.simxStartSimulation(API.clientID, sim.simx_opmode_blocking)#start simulation
            
            self.temp_x = []
            self.temp_y = []
            self.goalLoc =  sim.simxGetObjectPosition(API.clientID, API.goal, -1, sim.simx_opmode_blocking)
            
            totalReward = 0 #reward for episode is none initially 
            Done = False #if completed, this is set to true 
            Fail = False #if any failure, this will be true (for example, out of range)

            state_item = 0
            
            sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "sysCall_init",[] , [], [], bytearray(), sim.simx_opmode_blocking) #call initiation function 
            time.sleep(1)
            
            for i in range(iterations): #episodes are made of iterations, each iteration is an action of the agent 
                
                if(self.inRangeLocation() and not Done):
                    print("             {}%".format(int((i/iterations)*100)))
                    
                    previousLocation, state_item = self.getState() #the location and state prior to the action
                    noise_input = NoiseGenerator() #Generate value for noise

                    if random.uniform(0,1) < self.EP: #decides if agent performs random exploration action or if exploitation action
                        choice = random.randrange(2)
                        
                    else: 
                        try:
                            choice = np.argmax(self.q_table[state_item]) #choose optimal value for state, if available 
   
                        except:
                            print("Failure: Argmax(1)")
                            Fail = True
                            totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation, Fail)#provide penalty from failure
                            
                        
                    self.state_item = state_item    #set current state
                    self.action_space(choice,noise_input) #perform the action 
                    time.sleep(0.2) #sleep for action to be performed 
                    totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation,Fail)#get reward from action 
                    
                    newLocation, newState = self.getState() #update state and location
                    try:
                        newStateMax = np.max(self.q_table[newState]) #set current optimal action of state
                        self.q_table[state_item,choice] = (1-self.A)*self.q_table[state_item,choice]+self.A*(actionReward + self.G*newStateMax - self.q_table[state_item,choice])#update Q-Table
                            
                    except:
                       print("State Failure") 
                       
                else: 
                    print("Not In Range")
                    break
            
                
            sim.simxStopSimulation(API.clientID, sim.simx_opmode_blocking)#stop simulation
            
            API.TotalGraph.AddData(self.temp_x,self.temp_y,episode) #add episode to graph module 
            self.episodeReward.append(totalReward)
            print("             100%")
            print("Ended Simulation : {}".format(episode+1))
            print("Standby..")
            
            
            time.sleep(4) #Sleep time to allow for simulation to close properly, prevent overload
        
        Done = False  
        
        print("Running Final Sim")
        self.runModel() #run final model
        
        API.SecureClose() #Ensure closure 
        print("Total Rewards: {}".format(self.episodeReward))
        self.saveModel()
        print("Total Goals: {} Total Lefts: {} Total Rights: {}".format(self.GOAL, self.lefts, self.rights))
        


    def saveModel(self): #save model to the repo
        fileSave = open("model2.txt", "w")
        np.savetxt(fileSave, self.q_table)
        fileSave.close()

    
    def loadModel(self): #load model if not running q_table that are already imported 
        self.file = open("model2.txt", "r")
        self.q_table = np.genfromtxt(self.file)
        print(self.q_table)
        
    def runModel(self): #runs a simulation, with best action for all states
        Done = False 
        
        print("Starting Simulation")
        
        sim.simxStartSimulation(API.clientID, sim.simx_opmode_blocking)#start simulation
        self.temp_x = []
        self.temp_y = []
        self.goalLoc =  sim.simxGetObjectPosition(API.clientID, API.goal, -1, sim.simx_opmode_blocking)
        totalReward = 0
        Done = False
        Fail = False

        
        sim.simxCallScriptFunction(API.clientID, 'bristle', sim.sim_scripttype_customizationscript, "sysCall_init",[] , [], [], bytearray(), sim.simx_opmode_blocking)
        time.sleep(1)
        i = 0
        while(self.inRangeLocation() and not Done and i<200):
            if(self.inRangeLocation() and not Done):
                
                print("             {}%".format(int((i/300)*100)))
                
                noise_input = NoiseGenerator() #Generate value for noise
                previousLocation, state_item = self.getState()
                
                
                try: #always chooses best action for each known state (all states should be known)
                    choice = np.argmax(self.q_table[state_item])
                except:
                    print("Error E10000")
                    break
                
                self.action_space(choice,noise_input)
                time.sleep(0.5)    
                totalReward, actionReward, Done = self.rewardSystem(totalReward, previousLocation,Fail)
                
                
                i += 1
            else: 
                print("Not In Range")
                Done = True
        sim.simxStopSimulation(API.clientID, sim.simx_opmode_blocking)#stop simulation
        
        self.episodeReward.append(totalReward)
        print("             100%")
        print("Ended Simulation ")
        print("Standby..")    

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


EPISODES = 40
ITERATIONS = 45 ##30 might not be sufficient for goal reaching in most cases 

SPACE_INTERVAL = 0.1
X = [-1.5, 1.5]
Y = [-1.5, 1.5]


EPSILON = 0.15
ALPHA = 0.7
GAMMA = 0.7 #0.85


x = input("Training (1), Running (2):")
API = API()
Agent = Agent(SPACE_INTERVAL, EPSILON, ALPHA, GAMMA, X, Y)
 
if(x=="1"):
   Agent.RunCycle(EPISODES,ITERATIONS) #Run 10 simulations with 25 iterations each 
else: #will run a cycle 
    Agent.loadModel()
    Agent.runModel()
    Agent.file.close()

