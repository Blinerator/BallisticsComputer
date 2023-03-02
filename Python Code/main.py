import TrajectoryCalc
import SerialCOM

if 1==1:
    initial_velocity=40
    rocket_radius=21.6/1000 #sabot radius in meters
    rocket_mass=.162 #sabot mass in kg
    p=1.275 #density of fluid
    sample_resolution=0.005
    drag_coeff=.00
    y_displacement=0 #initial displacement of sabot in the "y" direction
    dir_ind='d'
    RangeFinder_COMPrt='COM21'
    Arduino_COMPrt='COM15'

while 1==1:
    a=input("Laze target? (y):\n")
    if a=='y':
        range=int(SerialCOM.takerange(RangeFinder_COMPrt)) #this is the linear range to target
        
        pitch_roll=SerialCOM.read_arduino_sensors(Arduino_COMPrt)
        normalized_pitch=pitch_roll[0]/200 #this is "theta" in right angled triangle with "range" being hypotenuse.  Also includes conversion to degrees
        roll=pitch_roll[1]/200 #conversion to degrees
        initial_velocity=50

        coords=TrajectoryCalc.find_x_y(range,normalized_pitch) #find "a" and "b" in right angled triangle, input in degrees.
        new_normalized_pitch=TrajectoryCalc.return_angle(coords[0],coords[1],dir_ind,initial_velocity,drag_coeff,rocket_radius,rocket_mass,sample_resolution,y_displacement,p) #this is the big boy
        delta_YawPitch=TrajectoryCalc.find_new_delta_yaw_pitch(new_normalized_pitch[0],roll) #how much to adjust base and elevation to get to new normalized pitch, input in degrees

        b=input(f"Range to target:{range}. Angle adjustments: yaw={delta_YawPitch[0]}, total pitch={delta_YawPitch[1]}, Theoretical error: {new_normalized_pitch[1]:.1f} meters.  Proceed? (y/n)")
        
        if b=='y':
            #print(int(delta_YawPitch[1]*11459.5))
            SerialCOM.write_to_arduino(int(delta_YawPitch[0]*381.983129),int(delta_YawPitch[1]*11459.5),0,1,Arduino_COMPrt)#including unit conversion
            SerialCOM.write_to_arduino(int(delta_YawPitch[0]*381.983129),int(delta_YawPitch[1]*11459.5),0,0,Arduino_COMPrt)#including unit conversion