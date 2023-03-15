# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 21:50:49 2023

@author: mplac
"""

import random
import matplotlib.pyplot as plt
import numpy as np
y = []
X = []
sum = 0
for x in range(500):
    variable = random.gauss(10,10)
    sign = random.randint(0,1)
    
    if(sign==0):
        pass
    else:
        variable = -variable
    
    
    original_input = 50
    change = 50*variable/100
    y.append(original_input+change)
    X.append(x)
    sum = sum+original_input+change

mean = sum/500
x_ = np.linspace(0,500,500)
y_ = x_ * 0 + mean



# =============================================================================
# plt.plot(X,y,label='Actual Set Motor Speed')
# plt.plot(x_,y_,label='Mean Value')
# plt.legend(loc='best')
# plt.ylim([40,60])
# plt.ylabel("Motor Speed Setting")
# plt.xlabel("Transmission")
# plt.show()
# =============================================================================

plt.hist(y)
plt.title("Value Distribution")
plt.xlabel("Motor Speed")
plt.ylabel("Frequency")