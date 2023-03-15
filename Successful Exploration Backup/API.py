# -*- coding: utf-8 -*-
"""
API for Coppeliasim 

V1.0 

Milosz Placzek


API Port: 19990

V-REP Folder: C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu
"""
try: 
    import sim #sim file associated with location of the API
    import random
    import matplotlib.pyplot as plt
except: 
    print("SIM loading failed. Please ensure sim.py is located in the same repository")

import time

print("Program Started")

sim.simxFinish(-1) # close any pre-exisiting connections 
time.sleep(2) # allowable time to ensure closure 

clientID=sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim
plt.axis([-1,1,-1,1])
if clientID!=-1:
    print("Connection Successful")
    returnCode, pingTime = sim.simxGetPingTime(clientID)
    print("Ping test speed:{}ms".format(pingTime))
    if(returnCode==0):{print("Connection Status:OK")}
    else:{print("Connection Status:FAILURE")}
    print("Simulation running...")
    
    
    error,handle = sim.simxGetObjectHandle(clientID, "Locator", sim.simx_opmode_oneshot_wait)
    
    data_x = ()
    data_x = list(data_x)
    
    
    data_y = ()
    data_y = list(data_y)
    
    for n in range(5):
        sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)#start simulation
        for i in range(25):
            
            noise_input = random.randrange(25)#generate noise as a random value 0-15
            noise_s = random.randrange(2)
            if(noise_s==0): noise_input=noise_input
            else:noise_input = -noise_input
            
            
            
            sim.simxCallScriptFunction(clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_right_50", [noise_input], [], [], bytearray(), sim.simx_opmode_blocking)
            location = sim.simxGetObjectPosition(clientID, handle, -1, sim.simx_opmode_blocking)
            
            temp_x = location[1][0]
            temp_y = location[1][1]        
            
            data_x.append(temp_x)
            data_y.append(temp_y)
            
            time.sleep(1)
            
            noise_input = random.randrange(25)#generate noise as a random value 0-15
            noise_s = random.randrange(2)
            if(noise_s==0): noise_input=noise_input
            else:noise_input = -noise_input
    
            sim.simxCallScriptFunction(clientID, 'bristle', sim.sim_scripttype_customizationscript, "speed_left_50", [noise_input], [], [], bytearray(), sim.simx_opmode_blocking)
            location = sim.simxGetObjectPosition(clientID, handle, -1, sim.simx_opmode_blocking)
            
            
            temp_x = location[1][0]
            temp_y = location[1][1]        
            
            data_x.append(temp_x)
            data_y.append(temp_y)
            
            
            time.sleep(1)
        
    
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)#stop simulation
     
    print("Terminating Connection")
    
    plt.plot(data_x,data_y)
    plt.tight_layout()
    plt.show()
    
        
    
        
else: 
    print("conneciton Failed. Ensure Port is set to 19990.")