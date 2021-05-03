// File in C++ format
// Application title: Mars Rover Simulation and Control
//
//    copyright (C) 2009 Olivier VERLINDEN
//    Service de Mecanique rationnelle, Dynamique et Vibrations
//    Faculte Polytechnique de Mons
//    31, Bd Dolez, 7000 MONS (Belgium)
//    Olivier.Verlinden@fpms.ac.be
//
// This file is part of EasyDyn
//
// EasyDyn is free software; you can redistribute it and/or modify it under the
// terms of the GNU GeneralPublic License as published by the Free Software
// Foundation; either version 2, or (at your option) any later version.
//
// EasyDyn is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// EasyDyn; see the file COPYING.  If not, write to the Free Software
// Foundation, 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

//

#define EASYDYNMBSMAIN  // to declare the global variable
#define EASYDYNMBSADVANCED // This option implies that a specific
                           // version of ComputePartialVelocities() is provided
#include <stdio.h>
#include <math.h>
#include <EasyDyn/mbs.h>
#include <EasyDyn/visu.h>
#include <EasyDyn/vec.h>
#include <fstream>

#define nActuator 4

using namespace std;
scene thescene;
ofstream VanFile;

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

//---------------------------------------------------

void WriteDataHeader(ostream &OutFile)
{
WriteStateVariablesHeader(OutFile);
OutFile << endl;
}

//----------------------------------------------------

void SaveData(ostream &OutFile)
{
///-----------------------///
/// Set point computation ///
///-----------------------///

Acceleration_Cst = DesiredVelocity/Acceleration_Time;

if(t<Acceleration_Time)
{
    Current_Position = (1.0/2.0) * Acceleration_Cst * pow(t,2.0);
    Current_Velocity = Acceleration_Cst * t;
    Current_Acceleration = Acceleration_Cst;
}
else
{
    Current_Position = (1.0/2.0) * Acceleration_Cst * pow(Acceleration_Time,2.0)+
                       DesiredVelocity * (t-Acceleration_Time);
    Current_Velocity = DesiredVelocity;
    Current_Acceleration = 0.0;
}

SetPoint = Current_Position;

///-------------------------------///
/// PI Controller: Forward motion ///
///-------------------------------///

Error_Current = Current_Position-q[0];
ControllerCommand = ((Ki_controller*dt+2.0*Kp_controller)*Error_Current+
                       (Ki_controller*dt-2.0*Kp_controller)*Error_Old+2.0*ControllerCommand_old)/2.0;

ControllerCommand_old=ControllerCommand;
Error_Old=Error_Current;

u[0]=ControllerCommand;

SaveStateVariables(OutFile);
OutFile << " " << Current_Position
        << " " << Current_Velocity
        << " " << Current_Acceleration
        << " " << Error_Current <<  endl;

/*Just saving the animation scene every SpacedFrame
to speed up animation in EasyAnim */
if( (SaveFrameCounter%SpacedFrame) == 0)
{
    thescene.WriteCoord(VanFile);
    SaveFrameCounter=0;
}

SaveFrameCounter++;
}

//----------------------------------------------------

void SetInertiaData()
{
body[0].mass=1.0e-10;
body[0].PhiG.Ixx=1.0e-10;
body[0].PhiG.Iyy=1.0e-10;
body[0].PhiG.Izz=1.0e-10;
body[0].PhiG.Ixy=0;
body[0].PhiG.Ixz=0;
body[0].PhiG.Iyz=0;
body[1].mass=42.600000000000001;
body[1].PhiG.Ixx=0.83999999999999997;
body[1].PhiG.Iyy=1.8100000000000001;
body[1].PhiG.Izz=2.4399999999999999;
body[1].PhiG.Ixy=0;
body[1].PhiG.Ixz=0;
body[1].PhiG.Iyz=0;
body[2].mass=2.5;
body[2].PhiG.Ixx=0.0030000000000000001;
body[2].PhiG.Iyy=0.0030000000000000001;
body[2].PhiG.Izz=0.0030000000000000001;
body[2].PhiG.Ixy=0;
body[2].PhiG.Ixz=0;
body[2].PhiG.Iyz=0;
body[3].mass=0.5;
body[3].PhiG.Ixx=0.001;
body[3].PhiG.Iyy=1.0000000000000001e-5;
body[3].PhiG.Izz=0.001;
body[3].PhiG.Ixy=0;
body[3].PhiG.Ixz=0;
body[3].PhiG.Iyz=0;
body[4].mass=2.0;
body[4].PhiG.Ixx=0.040000000000000001;
body[4].PhiG.Iyy=0.059999999999999998;
body[4].PhiG.Izz=0.040000000000000001;
body[4].PhiG.Ixy=0;
body[4].PhiG.Ixz=0;
body[4].PhiG.Iyz=0;
body[5].mass=2.5;
body[5].PhiG.Ixx=0.0030000000000000001;
body[5].PhiG.Iyy=0.0030000000000000001;
body[5].PhiG.Izz=0.0030000000000000001;
body[5].PhiG.Ixy=0;
body[5].PhiG.Ixz=0;
body[5].PhiG.Iyz=0;
body[6].mass=0.5;
body[6].PhiG.Ixx=0.001;
body[6].PhiG.Iyy=1.0000000000000001e-5;
body[6].PhiG.Izz=0.001;
body[6].PhiG.Ixy=0;
body[6].PhiG.Ixz=0;
body[6].PhiG.Iyz=0;
body[7].mass=2.0;
body[7].PhiG.Ixx=0.040000000000000001;
body[7].PhiG.Iyy=0.059999999999999998;
body[7].PhiG.Izz=0.040000000000000001;
body[7].PhiG.Ixy=0;
body[7].PhiG.Ixz=0;
body[7].PhiG.Iyz=0;
body[8].mass=2.5;
body[8].PhiG.Ixx=0.0030000000000000001;
body[8].PhiG.Iyy=0.0030000000000000001;
body[8].PhiG.Izz=0.0030000000000000001;
body[8].PhiG.Ixy=0;
body[8].PhiG.Ixz=0;
body[8].PhiG.Iyz=0;
body[9].mass=0.5;
body[9].PhiG.Ixx=0.001;
body[9].PhiG.Iyy=1.0000000000000001e-5;
body[9].PhiG.Izz=0.001;
body[9].PhiG.Ixy=0;
body[9].PhiG.Ixz=0;
body[9].PhiG.Iyz=0;
body[10].mass=2.0;
body[10].PhiG.Ixx=0.040000000000000001;
body[10].PhiG.Iyy=0.059999999999999998;
body[10].PhiG.Izz=0.040000000000000001;
body[10].PhiG.Ixy=0;
body[10].PhiG.Ixz=0;
body[10].PhiG.Iyz=0;
body[11].mass=2.5;
body[11].PhiG.Ixx=0.0030000000000000001;
body[11].PhiG.Iyy=0.0030000000000000001;
body[11].PhiG.Izz=0.0030000000000000001;
body[11].PhiG.Ixy=0;
body[11].PhiG.Ixz=0;
body[11].PhiG.Iyz=0;
body[12].mass=0.5;
body[12].PhiG.Ixx=0.001;
body[12].PhiG.Iyy=1.0000000000000001e-5;
body[12].PhiG.Izz=0.001;
body[12].PhiG.Ixy=0;
body[12].PhiG.Ixz=0;
body[12].PhiG.Iyz=0;
body[13].mass=2.0;
body[13].PhiG.Ixx=0.040000000000000001;
body[13].PhiG.Iyy=0.059999999999999998;
body[13].PhiG.Izz=0.040000000000000001;
body[13].PhiG.Ixy=0;
body[13].PhiG.Ixz=0;
body[13].PhiG.Iyz=0;
}

//-----------------------------------------

void ComputeMotion()
{
//Homogenous transformation matrices of each body
//Insert kinematics generated by python/sympy

//Body[0]
body[0].T0G.R.r11=1;
body[0].T0G.R.r12=0;
body[0].T0G.R.r13=0;
body[0].T0G.R.r21=0;
body[0].T0G.R.r22=1;
body[0].T0G.R.r23=0;
body[0].T0G.R.r31=0;
body[0].T0G.R.r32=0;
body[0].T0G.R.r33=1;
body[0].T0G.e.x=0;
body[0].T0G.e.y=0;
body[0].T0G.e.z=0;
//Body[1]
body[1].T0G.R.r11=cos(q[4])*cos(q[5]);
body[1].T0G.R.r12=-sin(q[5])*cos(q[4]);
body[1].T0G.R.r13=sin(q[4]);
body[1].T0G.R.r21=sin(q[3])*sin(q[4])*cos(q[5]) + sin(q[5])*cos(q[3]);
body[1].T0G.R.r22=-sin(q[3])*sin(q[4])*sin(q[5]) + cos(q[3])*cos(q[5]);
body[1].T0G.R.r23=-sin(q[3])*cos(q[4]);
body[1].T0G.R.r31=sin(q[3])*sin(q[5]) - sin(q[4])*cos(q[3])*cos(q[5]);
body[1].T0G.R.r32=sin(q[3])*cos(q[5]) + sin(q[4])*sin(q[5])*cos(q[3]);
body[1].T0G.R.r33=cos(q[3])*cos(q[4]);
body[1].T0G.e.x=q[0];
body[1].T0G.e.y=q[1];
body[1].T0G.e.z=q[2] + 0.28499999999999998;
body[1].vG.x=qd[0];
body[1].vG.y=qd[1];
body[1].vG.z=qd[2];
body[1].aG.x=qdd[0];
body[1].aG.y=qdd[1];
body[1].aG.z=qdd[2];
body[1].omega.x=qd[3] + qd[5]*sin(q[4]);
body[1].omega.y=qd[4]*cos(q[3]) - qd[5]*sin(q[3])*cos(q[4]);
body[1].omega.z=qd[4]*sin(q[3]) + qd[5]*cos(q[3])*cos(q[4]);
body[1].omegad.x=qd[4]*qd[5]*cos(q[4]) + qdd[3] + qdd[5]*sin(q[4]);
body[1].omegad.y=-qd[3]*qd[4]*sin(q[3]) - qd[3]*qd[5]*cos(q[3])*cos(q[4]) + qd[4]*qd[5]*sin(q[3])*sin(q[4]) + qdd[4]*cos(q[3]) - qdd[5]*sin(q[3])*cos(q[4]);
body[1].omegad.z=qd[3]*qd[4]*cos(q[3]) - qd[3]*qd[5]*sin(q[3])*cos(q[4]) - qd[4]*qd[5]*sin(q[4])*cos(q[3]) + qdd[4]*sin(q[3]) + qdd[5]*cos(q[3])*cos(q[4]);
//Body[2]
body[2].TrefG.R.r11=1;
body[2].TrefG.R.r12=0;
body[2].TrefG.R.r13=0;
body[2].TrefG.R.r21=0;
body[2].TrefG.R.r22=1;
body[2].TrefG.R.r23=0;
body[2].TrefG.R.r31=0;
body[2].TrefG.R.r32=0;
body[2].TrefG.R.r33=1;
body[2].TrefG.e.x=0.31;
body[2].TrefG.e.y=-0.13;
body[2].TrefG.e.z=-0.17000000000000001;
ComposeMotion(2,1);
//Body[3]
body[3].TrefG.R.r11=cos(50*q[6]);
body[3].TrefG.R.r12=0;
body[3].TrefG.R.r13=sin(50*q[6]);
body[3].TrefG.R.r21=0;
body[3].TrefG.R.r22=1;
body[3].TrefG.R.r23=0;
body[3].TrefG.R.r31=-sin(50*q[6]);
body[3].TrefG.R.r32=0;
body[3].TrefG.R.r33=cos(50*q[6]);
body[3].TrefG.e.x=0;
body[3].TrefG.e.y=0;
body[3].TrefG.e.z=0;
body[3].omegarel.y=50*qd[6];
body[3].omegadrel.y=50*qdd[6];
ComposeMotion(3,2);
//Body[4]
body[4].TrefG.R.r11=cos(q[6]);
body[4].TrefG.R.r12=0;
body[4].TrefG.R.r13=sin(q[6]);
body[4].TrefG.R.r21=0;
body[4].TrefG.R.r22=1;
body[4].TrefG.R.r23=0;
body[4].TrefG.R.r31=-sin(q[6]);
body[4].TrefG.R.r32=0;
body[4].TrefG.R.r33=cos(q[6]);
body[4].TrefG.e.x=0;
body[4].TrefG.e.y=-0.19;
body[4].TrefG.e.z=0;
body[4].omegarel.y=qd[6];
body[4].omegadrel.y=qdd[6];
ComposeMotion(4,3);
//Body[5]
body[5].TrefG.R.r11=1;
body[5].TrefG.R.r12=0;
body[5].TrefG.R.r13=0;
body[5].TrefG.R.r21=0;
body[5].TrefG.R.r22=1;
body[5].TrefG.R.r23=0;
body[5].TrefG.R.r31=0;
body[5].TrefG.R.r32=0;
body[5].TrefG.R.r33=1;
body[5].TrefG.e.x=0.31;
body[5].TrefG.e.y=0.13;
body[5].TrefG.e.z=-0.17000000000000001;
ComposeMotion(5,1);
//Body[6]
body[6].TrefG.R.r11=cos(50*q[7]);
body[6].TrefG.R.r12=0;
body[6].TrefG.R.r13=sin(50*q[7]);
body[6].TrefG.R.r21=0;
body[6].TrefG.R.r22=1;
body[6].TrefG.R.r23=0;
body[6].TrefG.R.r31=-sin(50*q[7]);
body[6].TrefG.R.r32=0;
body[6].TrefG.R.r33=cos(50*q[7]);
body[6].TrefG.e.x=0;
body[6].TrefG.e.y=0;
body[6].TrefG.e.z=0;
body[6].omegarel.y=50*qd[7];
body[6].omegadrel.y=50*qdd[7];
ComposeMotion(6,5);
//Body[7]
body[7].TrefG.R.r11=cos(q[7]);
body[7].TrefG.R.r12=0;
body[7].TrefG.R.r13=sin(q[7]);
body[7].TrefG.R.r21=0;
body[7].TrefG.R.r22=1;
body[7].TrefG.R.r23=0;
body[7].TrefG.R.r31=-sin(q[7]);
body[7].TrefG.R.r32=0;
body[7].TrefG.R.r33=cos(q[7]);
body[7].TrefG.e.x=0;
body[7].TrefG.e.y=0.19;
body[7].TrefG.e.z=0;
body[7].omegarel.y=qd[7];
body[7].omegadrel.y=qdd[7];
ComposeMotion(7,6);
//Body[8]
body[8].TrefG.R.r11=1;
body[8].TrefG.R.r12=0;
body[8].TrefG.R.r13=0;
body[8].TrefG.R.r21=0;
body[8].TrefG.R.r22=1;
body[8].TrefG.R.r23=0;
body[8].TrefG.R.r31=0;
body[8].TrefG.R.r32=0;
body[8].TrefG.R.r33=1;
body[8].TrefG.e.x=-0.28999999999999998;
body[8].TrefG.e.y=-0.13;
body[8].TrefG.e.z=-0.17000000000000001;
ComposeMotion(8,1);
//Body[9]
body[9].TrefG.R.r11=cos(50*q[8]);
body[9].TrefG.R.r12=0;
body[9].TrefG.R.r13=sin(50*q[8]);
body[9].TrefG.R.r21=0;
body[9].TrefG.R.r22=1;
body[9].TrefG.R.r23=0;
body[9].TrefG.R.r31=-sin(50*q[8]);
body[9].TrefG.R.r32=0;
body[9].TrefG.R.r33=cos(50*q[8]);
body[9].TrefG.e.x=0;
body[9].TrefG.e.y=0;
body[9].TrefG.e.z=0;
body[9].omegarel.y=50*qd[8];
body[9].omegadrel.y=50*qdd[8];
ComposeMotion(9,8);
//Body[10]
body[10].TrefG.R.r11=cos(q[8]);
body[10].TrefG.R.r12=0;
body[10].TrefG.R.r13=sin(q[8]);
body[10].TrefG.R.r21=0;
body[10].TrefG.R.r22=1;
body[10].TrefG.R.r23=0;
body[10].TrefG.R.r31=-sin(q[8]);
body[10].TrefG.R.r32=0;
body[10].TrefG.R.r33=cos(q[8]);
body[10].TrefG.e.x=0;
body[10].TrefG.e.y=-0.19;
body[10].TrefG.e.z=0;
body[10].omegarel.y=qd[8];
body[10].omegadrel.y=qdd[8];
ComposeMotion(10,9);
//Body[11]
body[11].TrefG.R.r11=1;
body[11].TrefG.R.r12=0;
body[11].TrefG.R.r13=0;
body[11].TrefG.R.r21=0;
body[11].TrefG.R.r22=1;
body[11].TrefG.R.r23=0;
body[11].TrefG.R.r31=0;
body[11].TrefG.R.r32=0;
body[11].TrefG.R.r33=1;
body[11].TrefG.e.x=-0.28999999999999998;
body[11].TrefG.e.y=0.13;
body[11].TrefG.e.z=-0.17000000000000001;
ComposeMotion(11,1);
//Body[12]
body[12].TrefG.R.r11=cos(50*q[9]);
body[12].TrefG.R.r12=0;
body[12].TrefG.R.r13=sin(50*q[9]);
body[12].TrefG.R.r21=0;
body[12].TrefG.R.r22=1;
body[12].TrefG.R.r23=0;
body[12].TrefG.R.r31=-sin(50*q[9]);
body[12].TrefG.R.r32=0;
body[12].TrefG.R.r33=cos(50*q[9]);
body[12].TrefG.e.x=0;
body[12].TrefG.e.y=0;
body[12].TrefG.e.z=0;
body[12].omegarel.y=50*qd[9];
body[12].omegadrel.y=50*qdd[9];
ComposeMotion(12,11);
//Body[13]
body[13].TrefG.R.r11=cos(q[9]);
body[13].TrefG.R.r12=0;
body[13].TrefG.R.r13=sin(q[9]);
body[13].TrefG.R.r21=0;
body[13].TrefG.R.r22=1;
body[13].TrefG.R.r23=0;
body[13].TrefG.R.r31=-sin(q[9]);
body[13].TrefG.R.r32=0;
body[13].TrefG.R.r33=cos(q[9]);
body[13].TrefG.e.x=0;
body[13].TrefG.e.y=0.19;
body[13].TrefG.e.z=0;
body[13].omegarel.y=qd[9];
body[13].omegadrel.y=qdd[9];
ComposeMotion(13,12);
}

//-----------------------------------------

void ComputePartialVelocities()
{
//Body[0]
//Body[1]
body[1].vGpartial[0].x=1;
body[1].vGpartial[1].y=1;
body[1].vGpartial[2].z=1;
body[1].omegapartial[3].x=1;
body[1].omegapartial[4].y=cos(q[3]);
body[1].omegapartial[4].z=sin(q[3]);
body[1].omegapartial[5].x=sin(q[4]);
body[1].omegapartial[5].y=-sin(q[3])*cos(q[4]);
body[1].omegapartial[5].z=cos(q[3])*cos(q[4]);
//Body[2]
ComposePartialVelocities(2,1);
//Body[3]
body[3].omegarelpartial[6].y=50;
ComposePartialVelocities(3,2);
//Body[4]
body[4].omegarelpartial[6].y=1;
ComposePartialVelocities(4,3);
//Body[5]
ComposePartialVelocities(5,1);
//Body[6]
body[6].omegarelpartial[7].y=50;
ComposePartialVelocities(6,5);
//Body[7]
body[7].omegarelpartial[7].y=1;
ComposePartialVelocities(7,6);
//Body[8]
ComposePartialVelocities(8,1);
//Body[9]
body[9].omegarelpartial[8].y=50;
ComposePartialVelocities(9,8);
//Body[10]
body[10].omegarelpartial[8].y=1;
ComposePartialVelocities(10,9);
//Body[11]
ComposePartialVelocities(11,1);
//Body[12]
body[12].omegarelpartial[9].y=50;
ComposePartialVelocities(12,11);
//Body[13]
body[13].omegarelpartial[9].y=1;
ComposePartialVelocities(13,12);
}

//-----------------------------------------

void AddAppliedEfforts()
{
//Contribution of gravity
vec gravity(0,0,-3.7200000000000002);
AddGravityForces(gravity);

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
}

//-----------------------------------------

void ComputeResidual()
{
ComputeResidualmbs();
}

//-----------------------------------------

int main()
{
// Initialization and memory allocation
nbrdof=10;
nbrdep=0;
nbrbody=14;

/// Input for motor
nbrinput = 1;

application=new char[7];
strcpy(application,"Rover");
InitEasyDynmbs();
// Let's create the shapes
shape *s0_Ground;
s0_Ground=new box(&(body[0].T0G),vcoord(-5.0,-5.0,0.0),vcoord(5.0,5.0,-0.01),0,6);
thescene.AddShape(s0_Ground);

shape *s1_Chassis;
s1_Chassis=new habfile(&(body[1].T0G),"3Dshapes/MainBody_Sld2017_Simplified_Light.hab");
thescene.AddShape(s1_Chassis);

/// Front motor left
shape *s2_LeftMotor;
s2_LeftMotor=new habfile(&(body[2].T0G),"3Dshapes/Motor_Sld2017_Left_Light.hab");
thescene.AddShape(s2_LeftMotor);
shape *s3_LeftRotor;
s3_LeftRotor=new habfile(&(body[3].T0G),"3Dshapes/MotorShaft_Sld2017_Left_Light.hab");
thescene.AddShape(s3_LeftRotor);
shape *s4_LeftWheel;
s4_LeftWheel=new habfile(&(body[4].T0G),"3Dshapes/Wheel_Sld2017_Left_Light.hab");
thescene.AddShape(s4_LeftWheel);

/// Front motor right
shape *s5_RightMotor;
s5_RightMotor=new habfile(&(body[5].T0G),"3Dshapes/Motor_Sld2017_Right_Light.hab");
thescene.AddShape(s5_RightMotor);
shape *s6_RightRotor;
s6_RightRotor=new habfile(&(body[6].T0G),"3Dshapes/MotorShaft_Sld2017_Right_Light.hab");
thescene.AddShape(s6_RightRotor);
shape *s7_RightWheel;
s7_RightWheel=new habfile(&(body[7].T0G),"3Dshapes/Wheel_Sld2017_Right_Light.hab");
thescene.AddShape(s7_RightWheel);

/// Rear motor left
shape *s8_LeftMotor;
s8_LeftMotor=new habfile(&(body[8].T0G),"3Dshapes/Motor_Sld2017_Left_Light.hab");
thescene.AddShape(s8_LeftMotor);
shape *s9_LeftRotor;
s9_LeftRotor=new habfile(&(body[9].T0G),"3Dshapes/MotorShaft_Sld2017_Left_Light.hab");
thescene.AddShape(s9_LeftRotor);
shape *s10_LeftWheel;
s10_LeftWheel=new habfile(&(body[10].T0G),"3Dshapes/Wheel_Sld2017_Left_Light.hab");
thescene.AddShape(s10_LeftWheel);

/// Rear motor right
shape *s11_RightMotor;
s11_RightMotor=new habfile(&(body[11].T0G),"3Dshapes/Motor_Sld2017_Right_Light.hab");
thescene.AddShape(s11_RightMotor);
shape *s12_RightRotor;
s12_RightRotor=new habfile(&(body[12].T0G),"3Dshapes/MotorShaft_Sld2017_Right_Light.hab");
thescene.AddShape(s12_RightRotor);
shape *s13_RightWheel;
s13_RightWheel=new habfile(&(body[13].T0G),"3Dshapes/Wheel_Sld2017_Right_Light.hab");
thescene.AddShape(s13_RightWheel);
// Uncomment the following line if you want a moving observer
// thescene.SetVisuFrame(&Tref);
// Let's open an animation file
VanFile.open("Rover.van");
// Initial configuration

/* Need to be determined for the static equilibrium
 Method: Let the system stabilize without calling
 StaticEquilibrium function and plot the graph with
 the degrees of freedom and see how much q[2] (elevation
 of the rover) was decreased, then put the value
 as initial condition for q[2]
 Call StaticEquilibrium() function
 q[2] = -0.01 also works, as long as the
 the tire is a little bit inside the ground */

q[2] = -5e-5;

// Let's save the structure of the scene
ComputeMotion();
thescene.CreateVolFile("Rover.vol");
// Searching for equilibrium position
StaticEquilibrium();

// Let's calculate the poles
cout << "Eigen value analysis" << endl;
SaveLinearizedSystem();
ComputePoles();
CreateVmoFile(thescene);
cout << "Eigen values computed !" << endl;

// Let's perform the integration !
NewmarkIntegration(t_final,dt,1e-5);
// The clean way to finish
EndEasyDynmbs();
VanFile.close();

}
//-----------------------------------------
