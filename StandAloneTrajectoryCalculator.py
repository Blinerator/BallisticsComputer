import math
import matplotlib.pyplot as plt
import numpy as np
import os

os.system('CLS')

initial_velocity=55
rocket_radius=10/1000 #projectile radius in meters
rocket_mass=.1 #projectile mass in kg

sample_resolution=0.1
drag_coeff=0.15 #taken from "Golf Ball Flight Dynamics" by Brett Burglund and Ryan Street

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

def eulers_method(p,v,D,r,m,time_interval,displacement,elevation):
    #print("Initializing Euler's Method for 'y' displacement...")
    start_displacement=displacement
    y_array=[]

    v_y=v*math.sin(math.radians(elevation)) #calculating components of velocity
    v_y_forprint=v_y
    v_x=v*math.cos(math.radians(elevation))
    v_x_forprint=v_x
    max=0
    while(displacement>=start_displacement):
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

ideal_x=int(input("Enter desired range"))
ideal_y=int(input("Enter desired height"))
dir_ind=str(input("Select direct fire (d) or indirect fire (i)"))


if dir_ind == 'd': #max range is achieved at around 40 degrees, therefore anything greater will be indirect fire
    start_angle=0
    end_angle=37.8
else:
    start_angle=37.8
    end_angle=90

prev_margin=np.inf

for i in np.arange(start_angle,end_angle,0.1): #going through every possible firing solution
    eulers_output=eulers_method(1.275,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,0,i) # [0] is time in flight (seconds), [1] is maximum horizontal range, [2] is x_array, [3] is y_array.  Both x & y arrays are displacement coordinates. [4] is "x" v, [5] is "y" v, [6] is y_max, [7] is y v, [8] is x v
    
    prev_x_margin=np.inf
    
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

print(f"Angle from horizontal: {theta:.1f} degrees, Theoretical error: {prev_margin:.1f} meters")

eulers_output=eulers_method(1.275,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,0,theta)

#fin_vel=np.sqrt(eulers_output[7]*eulers_output[7]+eulers_output[8]*eulers_output[8]) #this only tells you velocity at y=0
#print(fin_vel)

with open('Coordinate_Log.txt','w') as f:
    f.write('Flight Time: %f seconds, Max Height: %f meters, Max Range: %f meters, (x,y) initial velocity: (%f,%f)|'%(eulers_output[0], eulers_output[6], eulers_output[1], eulers_output[4], eulers_output[5]))
    f.write('\n\n')
    f.write(str(eulers_output[2]))
    f.write('\n\n')
    f.write(str(eulers_output[3]))

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
