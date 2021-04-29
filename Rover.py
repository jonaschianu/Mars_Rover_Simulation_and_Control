#!/usr/bin/python3

import sys
sys.path.insert(0, '../../../..')
sys.dont_write_bytecode = True # Disable pycache folder

from cagem import * # Import functions to derive system kinematics

#~~~~~~~#
# Mars Rover Simulation and Control #
#~~~~~~~#

#-------------------#
# System properties #
#-------------------#
Rvr=MBSClass(nbrbody=14,\
				nbrdof=10,\
				nbrdep=0,\
			    ApplicationTitle="Mars Rover Simulation and Control",\
			    ApplicationFileName="Rover")				
# nbrbody: Number of bodies
# nbrdof:  Number of degrees of freedom
# nbrdep:  Number of dependent parameters			
q,p,pexpr,t=Rvr.Getqpt() # Declare system variables

# Gravity vector

# Mars Gravity = -3.72 m/s^2
Rvr.SetGravity(0,0,-3.72) 

# Write in cpp format
# Set global variables
Rvr.SetGlobalVar("""
/* Motor constants */
double n=50.0; // Reduction ratio
double Kt[nActuator] = {0.4,0.4,0.4,0.4};
double Ra[nActuator] = {0.85,0.85,0.85,0.85};
double i_current[nActuator] = {0.0,0.0,0.0,0.0};

// Trajectory profile
double Acceleration_Time = 1.0; // [s]
double DesiredVelocity = 0.4; // [m/s]
double SetPoint = 0.0;

double Acceleration_Cst = 0.0;
double Current_Position = 0.0;
double Current_Velocity = 0.0;
double Current_Acceleration = 0.0;

double t_final = 10.0; // Simulation time
double dt = 1e-3; // Time step

// Discrete PI controller
double Ki_controller = 100.0;
double Kp_controller = 1500.0;
double Error_Current = 0.0;
double Error_Old = 0.0;
double ControllerCommand = 0.0;
double ControllerCommand_old = 0.0;

// Animation speed up
int SaveFrameCounter = 0;
int SpacedFrame = 100; // Every X passages in SaveData, save the frame for EasyAnim
""")

# Some constants
n=50 # Reduction ratio

#--------------------#
# Inertia properties #
#--------------------#
Rvr.body[0].Set( mass=1e-10,Ixx=1e-10, Iyy=1e-10, Izz=1e-10) # Ground
Rvr.body[1].Set( mass=42.6 ,Ixx=0.84 , Iyy=1.81 , Izz=2.44)  # Main Body
Rvr.body[2].Set( mass=2.5  ,Ixx=0.003, Iyy=0.003, Izz=0.003) # Left  Motor 1 Front
Rvr.body[3].Set( mass=0.5  ,Ixx=0.001, Iyy=1e-5 , Izz=0.001) # Left  Rotor 1 Front
Rvr.body[4].Set( mass=2.0  ,Ixx=0.04 , Iyy=0.06 , Izz=0.04)  # Left  Wheel 1 Front
Rvr.body[5].Set( mass=2.5  ,Ixx=0.003, Iyy=0.003, Izz=0.003) # Right Motor 1 Front
Rvr.body[6].Set( mass=0.5  ,Ixx=0.001, Iyy=1e-5 , Izz=0.001) # Right Rotor 1 Front
Rvr.body[7].Set( mass=2.0  ,Ixx=0.04 , Iyy=0.06 , Izz=0.04)  # Right Wheel 1 Front
Rvr.body[8].Set( mass=2.5  ,Ixx=0.003, Iyy=0.003, Izz=0.003) # Left  Motor 2 Rear
Rvr.body[9].Set( mass=0.5  ,Ixx=0.001, Iyy=1e-5 , Izz=0.001) # Left  Rotor 2 Rear
Rvr.body[10].Set(mass=2.0  ,Ixx=0.04 , Iyy=0.06 , Izz=0.04)  # Left  Wheel 2 Rear
Rvr.body[11].Set(mass=2.5  ,Ixx=0.003, Iyy=0.003, Izz=0.003) # Right Motor 2 Rear
Rvr.body[12].Set(mass=0.5  ,Ixx=0.001, Iyy=1e-5 , Izz=0.001) # Right Rotor 2 Rear
Rvr.body[13].Set(mass=2.0  ,Ixx=0.04 , Iyy=0.06 , Izz=0.04)  # Right Wheel 2 Rear


#-------------------#
# Position matrices #
#-------------------#

# Ground
Rvr.body[0].T0F=Tdisp(0,0,0)
# Main body
# Frame of body 1 is free to move at the center of mass of the chassis
Rvr.body[1].T0F=Tdisp(q[0],q[1],q[2]+0.285)*Trotx(q[3])*Troty(q[4])*Trotz(q[5])

# Left Motor 1 Front
Rvr.body[2].TrefF=Tdisp(0.31,-0.13,-0.17)
Rvr.body[2].ReferenceFrame(1)
# Left Rotor 1 Front
Rvr.body[3].TrefF=Troty(n*q[6]) 
Rvr.body[3].ReferenceFrame(2)
# Left Wheel 1 Front
Rvr.body[4].TrefF=Tdisp(0,-0.19,0)*Troty(q[6])
Rvr.body[4].ReferenceFrame(3)

# Right Motor 1 Front
Rvr.body[5].TrefF=Tdisp(0.31,0.13,-0.17)
Rvr.body[5].ReferenceFrame(1)
# Right Rotor 1 Front
Rvr.body[6].TrefF=Troty(n*q[7])
Rvr.body[6].ReferenceFrame(5)
# Right Wheel 1 Front
Rvr.body[7].TrefF=Tdisp(0,0.19,0)*Troty(q[7])
Rvr.body[7].ReferenceFrame(6)

# Left Motor 2 Rear
Rvr.body[8].TrefF=Tdisp(-0.29,-0.13,-0.17)
Rvr.body[8].ReferenceFrame(1)
# Left Rotor 2 Rear
Rvr.body[9].TrefF=Troty(n*q[8])
Rvr.body[9].ReferenceFrame(8)
# Left Wheel 2 Rear
Rvr.body[10].TrefF=Tdisp(0,-0.19,0)*Troty(q[8])
Rvr.body[10].ReferenceFrame(9)

# Right Motor 2 Rear
Rvr.body[11].TrefF=Tdisp(-0.29,0.13,-0.17)
Rvr.body[11].ReferenceFrame(1)
# Right Rotor 2 Rear
Rvr.body[12].TrefF=Troty(n*q[9])
Rvr.body[12].ReferenceFrame(11)
# Right Wheel 2 Rear
Rvr.body[13].TrefF=Tdisp(0,0.19,0)*Troty(q[9])
Rvr.body[13].ReferenceFrame(12)

#-----------------------#
# Initial configuration #
#-----------------------#

Rvr.qini[0]=0 # [m/rad]
Rvr.qini[1]=0 # [m/rad]

# Need to be determined for the static equilibrium
# Method: Let the system stabilize without calling
# StaticEquilibrium function and plot the graph with
# the degrees of freedom and see how much q[2] (elevation
# of the rover) was decreased, then put the value
# as initial condition for q[2]
# Call StaticEquilibrium() function
# q[2] = -0.01 also works, as long as the
# the tire is a little bit inside the ground
# as indicated in the exam hint
Rvr.qini[2]=0 # [m/rad]

Rvr.qini[3]=0 # [m/rad]
Rvr.qini[4]=0 # [m/rad]
Rvr.qini[5]=0 # [m/rad]
Rvr.qini[6]=0 # [m/rad]
Rvr.qini[7]=0 # [m/rad]
Rvr.qini[8]=0 # [m/rad]
Rvr.qini[9]=0 # [m/rad]

#---------------------------#
# Forces #
#---------------------------#
Rvr.Force("""

///-----------///
/// Actuators ///
///-----------///

/// Actuator current
for(int i_motor=0; i_motor<nActuator; i_motor++)
    i_current[i_motor] = (u[0] - Kt[i_motor] * n*qd[6+i_motor])/Ra[i_motor];

/// Torques applied on the rotors

body[3].MG+= (Kt[0]*i_current[0])* body[3].T0G.R.uy();
body[2].MG-= (Kt[0]*i_current[0])* body[3].T0G.R.uy();

body[6].MG+= (Kt[1]*i_current[1])* body[6].T0G.R.uy();
body[5].MG-= (Kt[1]*i_current[1])* body[6].T0G.R.uy();

body[9].MG+= (Kt[2]*i_current[2])* body[9].T0G.R.uy();
body[8].MG-= (Kt[2]*i_current[2])* body[9].T0G.R.uy();

body[12].MG+= (Kt[3]*i_current[3])* body[12].T0G.R.uy();
body[11].MG-= (Kt[3]*i_current[3])* body[12].T0G.R.uy();

///-------///
/// Tires ///
///-------///

/* Tire data */
structtyre tyre_rover_Front;
tyre_rover_Front.r1=0.115;
tyre_rover_Front.r2=0.03;
tyre_rover_Front.Kz=1000000.0;
tyre_rover_Front.Cz=600.0;
// Total mass of rover = 62.6
tyre_rover_Front.Fznom=  (62.6*3.72)*0.29/(0.29+0.31)/2.0; // For one front wheel

tyre_rover_Front.Clongnom=1000.0;
tyre_rover_Front.nlong=0.1;
tyre_rover_Front.Clatnom=1000.0;
tyre_rover_Front.nlat=0.1;
tyre_rover_Front.Ccambernom=100.0;
tyre_rover_Front.ncamber=0.1;
tyre_rover_Front.fClbs=0.6;
tyre_rover_Front.fClbd=0.4;

structtyre tyre_rover_Rear;
tyre_rover_Rear.r1=0.115;
tyre_rover_Rear.r2=0.03;
tyre_rover_Rear.Kz=1000000.0;
tyre_rover_Rear.Cz=600.0;
// Total mass of rover = 62.6
tyre_rover_Rear.Fznom=  (62.6*3.72)*0.31/(0.29+0.31)/2.0; // For one rear wheel
tyre_rover_Rear.Clongnom=1000.0;
tyre_rover_Rear.nlong=0.1;
tyre_rover_Rear.Clatnom=1000.0;
tyre_rover_Rear.nlat=0.1;
tyre_rover_Rear.Ccambernom=100.0;
tyre_rover_Rear.ncamber=0.1;
tyre_rover_Rear.fClbs=0.6;
tyre_rover_Rear.fClbd=0.4;

/* Definition of the tire forces */
AddTyreEfforts(4,vcoord(0,1,0),tyre_rover_Front); /// Left Wheel 1 Front
AddTyreEfforts(7,vcoord(0,1,0),tyre_rover_Front); /// Right Wheel 1 Front

AddTyreEfforts(10,vcoord(0,1,0),tyre_rover_Rear); /// Left Wheel 2 Rear
AddTyreEfforts(13,vcoord(0,1,0),tyre_rover_Rear); /// Right Wheel 2 Rear

""")

#------------------------#
# Integration parameters #
#------------------------#
Rvr.SetIntegrationParameters(tfinal=5, hsave=0.01, hmax=0.005)
# tfinal = duration of simulation [s]
# hsave = time step for saving results [s]
# hmax = adaptive integration time step [s]

#---------#
# Options #
#---------#
STATIC=1 # Set STATIC to 1 to search the static equilibrium before integration
POLE=1 # Set POLE to 1 to perform an eigen value analysis
TEST=0 # Set TEST to 1 to perform the efficiency tests

Rvr.ComputeKinematics() # Derive the system kinematics symbolically
Rvr.EasyDynFlags(STATIC,POLE,TEST) # Define flags
Rvr.ExportEasyDynProgram() # Write .cpp file enclosing the kinematics

Rvr.ExportUK_Latex_Report() # Uncomment to generate the English LaTeX report
Rvr.ExportGnuplotScript() # Uncomment to output script for graphs of variables
