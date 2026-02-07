# A basic python code to implement the kinematics model for the differential mobile robot
# and reflect its behavior on graphs.

# import the required libraries:
import numpy as np
import matplotlib.pyplot as plt

# Define the inputs to the motors speeds:
omega_r =10 #the angular velocity of the right wheel
omega_l = 5 #the angular velocity of the left wheel

# Define the initial pose of the robot:
x0 = 0 #initial position in X-direction
y0 = 0 #initial position in Y-direction
theta0 = 0 #initial orientation around Z-axis

# Define the robot parameters required for simulation:
r = 0.2 #the wheel radius in meters
l = 0.2 #the wheel base distance in meters (width of robot)

# Define the time for simulation considered
Total_time = 10.00 #total time of simulatio
Ts = 0.01 #sampling time
Time = np.arange(0,Total_time + Ts,Ts)  #define the array of time steps

#Initialie the arrays to be used:
Num_pnts = len(Time)  #number of points in time array
x_dot = [0] * Num_pnts  #initialize an empty array
y_dot = [0] * Num_pnts  #initialize an empty array
theta_dot = [0] * Num_pnts  #initialize an empty array
x_vals = [0] * Num_pnts  #initialize an empty array
y_vals = [0] * Num_pnts  #initialize an empty array
theta_vals = [0] * Num_pnts  #initialize an empty array

# Executing the equations in a loop manner:
i = 0; #the element considered in each iteration
theta = theta0  #initial theta is used in calculations
for t in Time:
    theta_rad = np.pi * (theta/180.0)  #convert to radians instead of degrees
    x_dot[i] = (r*np.cos(theta_rad)/2) * (omega_r + omega_l)  #calculate xdot
    y_dot[i] = (r*np.sin(theta_rad)/2) * (omega_r + omega_l)  #calculate ydot
    theta_dot[i] = (r/l) * (omega_r - omega_l)  #calculate theta dot

    #update the pose:
    if i == 0:
        x_vals[i] = x0   #update position in X-direction
        y_vals[i] = y0   #update position in X-direction
        theta_vals[i] = theta0   #update position in X-direction
    else:
        x_vals[i] = x_vals[i-1] + x_dot[i]*Ts   #update position in X-direction
        y_vals[i] = y_vals[i-1] + y_dot[i]*Ts   #update position in X-direction
        theta_vals[i] = theta_vals[i-1] + theta_dot[i]*Ts   #update position in X-direction

    theta = theta_vals[i]  #update the current theta value considered    
    i = i + 1  #update the element considered
    

plt.plot(Time,x_vals,label = 'X-pos (m)', linewidth = 3)
plt.plot(Time,y_vals,label = 'Y-pos (m)', linewidth = 3)
#plt.plot(Time,theta_vals,label = 'Theta (deg)', linewidth = 3)
#plt.ylim(top = 12)
plt.xlabel('Time (seconds)')
plt.ylabel('States Behavior')
plt.title('Kinematic model Results')
plt.legend()
plt.grid()
plt.show()
