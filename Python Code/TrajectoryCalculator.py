from logging import exception
import math
import matplotlib.pyplot as plt
import numpy as np

def eulers_method_x(p,v_x,D,r,m,time_interval,displacement,time_till_touchdown):
    #print("Initializing Euler's Method for 'x' displacement...")

    array_x = []
    array_x_velocity=[]
    while(time_till_touchdown>0):
        a=-0.5*(p*v_x*v_x*D*3.1415*r*r)/m  #acceleration equation
        v_x=v_x+a*time_interval         #this calculates the NEW initial velocity for each time interval
        array_x_velocity.append(v_x)
        displacement=displacement+(v_x*time_interval+0.5*a*time_interval*time_interval) #this adds the displacement from each time interval to the last one
        time_till_touchdown=time_till_touchdown-1

        array_x.append(displacement)
    #print("Calculations completed successfully.  Please refer to text document for values.")
    return array_x, v_x

def eulers_method(p,v,D,r,m,time_interval,displacement,elevation,end_elev):
    #print("Initializing Euler's Method for 'y' displacement...")
    start_displacement=displacement
    y_array=[]

    v_y=v*math.sin(math.radians(elevation)) #calculating components of velocity
    v_y_forprint=v_y
    v_x=v*math.cos(math.radians(elevation))
    v_x_forprint=v_x
    max=0
    while(displacement>=end_elev):
        a=(-0.5*(p*v_y*v_y*D*3.1415*r*r)-(m*9.81))/m  #acceleration equation, this time with gravity
         
        displacement=displacement+(v_y*time_interval+0.5*a*time_interval*time_interval) #this adds the displacement from each time interval to the last one
        if displacement>max:
            max=displacement
        v_y=v_y+a*time_interval

        y_array.append(displacement)

    time_till_touchdown=len(y_array) #number of time intervals allow keeping x and y arrays the same dimensions 

    x_output=eulers_method_x(p,v_x,D,r,m,time_interval,start_displacement,time_till_touchdown)
    x_array=x_output[0]
    v_x=x_output[1]

    #print("Calculations completed successfully.  Please refer to text document for values.")
    return time_till_touchdown*time_interval, x_array[time_till_touchdown-1], x_array, y_array, v_x_forprint, v_y_forprint, max, v_y, v_x

def return_angle(ideal_x,ideal_y,dir_ind,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,y_displacement,p):
    if(ideal_y<0):
        end_elev=ideal_y-1
    else: end_elev=0

    prev_margin=np.inf
    if dir_ind == 'd': #max range is achieved at around 40 degrees, therefore anything greater will be indirect fire
        start_angle=0
        end_angle=37.5
    elif dir_ind == 'i':
        start_angle=37
        end_angle=90
    else:
        raise Exception(f"Invalid input for variable 'dir_ind'")

    for i in np.arange(start_angle,end_angle,.1): #going through every possible firing solution
        prev_x_margin=np.inf
        eulers_output=eulers_method(p,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,y_displacement,i,end_elev) # [0] is time in flight (seconds), [1] is maximum horizontal range, [2] is x_array, [3] is y_array.  Both x & y arrays are displacement coordinates. [4] is "x" v, [5] is "y" v, [6] is y_max, [7] is y v, [8] is x v
        for idx_element, x_element in enumerate(eulers_output[2]): #finding the closest coordinate to the ideal coordinate in the calculated trajectory
            x_margin=abs(ideal_x-x_element)
            if x_margin<prev_x_margin:
                prev_x_margin=x_margin
                closest_value_index=idx_element
        
        x_value=eulers_output[2][closest_value_index]
        y_value=eulers_output[3][closest_value_index]
        
        margin=abs((ideal_x+ideal_y)-(x_value+y_value)) #finding the closest coordinate out of all possible trajectories' coordinates
        if margin<prev_margin:
            prev_margin=margin
            theta=i
    
    eulers_output=eulers_method(p,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,y_displacement,theta,end_elev) # [0] is time in flight (seconds), [1] is maximum horizontal range, [2] is x_array, [3] is y_array.  Both x & y arrays are displacement coordinates. [4] is "x" v, [5] is "y" v, [6] is y_max, [7] is y v, [8] is x v

    if 1==1: #plotting graph 
        fig, (ax1,ax2) = plt.subplots(1,2)
        ax1.plot(eulers_output[2],eulers_output[3])
        ax1.plot(ideal_x,ideal_y,marker="x", markersize=20, markeredgecolor="red", markerfacecolor="red")
        plt.xlim([0,eulers_output[1]+5])
        plt.ylim([0,eulers_output[6]+5])
        ax2.plot(eulers_output[2],eulers_output[3])
        ax2.plot(ideal_x,ideal_y,marker="x", markersize=15, markeredgecolor="red", markerfacecolor="red")
        plt.xlim([0,eulers_output[1]+5])
        plt.ylim([0,eulers_output[1]+5])
        plt.show()
    return theta, prev_margin

def find_x_y(range,theta):
    x=range*math.cos(math.radians(theta))
    y=range*math.sin(math.radians(theta))
    return x,y

def find_new_delta_yaw_pitch(new_normalized_pitch,roll):
    new_normalized_pitch=math.radians(new_normalized_pitch)
    roll=math.radians(roll)
    new_yaw=math.asin(math.tan(roll)*math.tan(math.asin(math.cos(roll)*math.sin(new_normalized_pitch)))) #derived equ.
    delta_yaw=new_yaw #amount angle needs to change to get to new yaw location
    
    new_pitch=math.asin(math.cos(roll)*math.sin(new_normalized_pitch)) #derived equ.
    delta_pitch=new_pitch #amount angle needs to change to get to new pitch (NOT normalized, this value is ready-to-use) location
    
    return delta_yaw, delta_pitch 
    #Note: if this direct method doesn't work, another solution is to use delta_yaw, but let the arduino determine
    #delta_pitch based off the raw accelerometer measurement of pitch.  So just use pitch measurement to get as close
    #to ideal pitch measurement (both normalized).
