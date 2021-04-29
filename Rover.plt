reset
set xlabel "Time [s]" 
set grid 

set term postscript eps color "Times-Roman" 20  
set output "figure1.eps" 
set ylabel "displacements" 
plot 'Rover.res' using 1:2 title 'q_0' with line , 'Rover.res' using 1:5 title 'q_1' with line , 'Rover.res' using 1:8 title 'q_2' with line , 'Rover.res' using 1:11 title 'q_3' with line , 'Rover.res' using 1:14 title 'q_4' with line , 'Rover.res' using 1:17 title 'q_5' with line , 'Rover.res' using 1:20 title 'q_6' with line , 'Rover.res' using 1:23 title 'q_7' with line , 'Rover.res' using 1:26 title 'q_8' with line , 'Rover.res' using 1:29 title 'q_9' with line 
set term pop 
replot 
pause -1 'Next plot (velocity level)?' 

set term postscript eps color "Times-Roman" 20  
set output "figure2.eps" 
set ylabel "velocities" 
plot 'Rover.res' using 1:3 title 'qd_0' with line , 'Rover.res' using 1:6 title 'qd_1' with line , 'Rover.res' using 1:9 title 'qd_2' with line , 'Rover.res' using 1:12 title 'qd_3' with line , 'Rover.res' using 1:15 title 'qd_4' with line , 'Rover.res' using 1:18 title 'qd_5' with line , 'Rover.res' using 1:21 title 'qd_6' with line , 'Rover.res' using 1:24 title 'qd_7' with line , 'Rover.res' using 1:27 title 'qd_8' with line , 'Rover.res' using 1:30 title 'qd_9' with line 
set term pop 
replot 
pause -1 'Next plot (acceleration level)?' 

set term postscript eps color "Times-Roman" 20  
set output "figure3.eps" 
set ylabel "accelerations" 
plot 'Rover.res' using 1:4 title 'qdd_0' with line , 'Rover.res' using 1:7 title 'qdd_1' with line , 'Rover.res' using 1:10 title 'qdd_2' with line , 'Rover.res' using 1:13 title 'qdd_3' with line , 'Rover.res' using 1:16 title 'qdd_4' with line , 'Rover.res' using 1:19 title 'qdd_5' with line , 'Rover.res' using 1:22 title 'qdd_6' with line , 'Rover.res' using 1:25 title 'qdd_7' with line , 'Rover.res' using 1:28 title 'qdd_8' with line , 'Rover.res' using 1:31 title 'qdd_9' with line 
set term pop 
replot 
pause -1 'Next plot (Setpoint displacement level)?'

set term postscript eps color "Times-Roman" 20  
set output "Setpoint_x.eps" 
set ylabel "displacements" 
plot 'Rover.res' using 1:33 title 'x' with line 
set term pop 
replot 
pause -1 'Next plot (Setpoint velocity level)?' 

set term postscript eps color "Times-Roman" 20  
set output "Setpoint_xd.eps" 
set ylabel "velocities" 
plot 'Rover.res' using 1:34 title 'xd' with line 
set term pop 
replot 
pause -1 'Next plot (Setpoint acceleration level)?' 

set term postscript eps color "Times-Roman" 20  
set output "Setpoint_xdd.eps" 
set ylabel "accelerations" 
plot 'Rover.res' using 1:35 title 'xdd' with line 
set term pop 
replot 
pause -1 'Next plot (Tracking)?'

set term postscript eps color "Times-Roman" 20  
set output "Tracking_x.eps" 
set ylabel "displacements" 
plot 'Rover.res' using 1:33 title 'x' with line , 'Rover.res' using 1:2 title 'q0' with line
set term pop 
replot 
pause -1 'Next plot (Error)?' 

set term postscript eps color "Times-Roman" 20  
set output "Error.eps" 
set ylabel "Error [m]" 
plot 'Rover.res' using 1:36 title 'xd' with line 
set term pop 
replot 
pause -1 'Next plot (Voltage)?' 

set term postscript eps color "Times-Roman" 20  
set output "Voltage.eps" 
set ylabel "Voltage [V]" 
plot 'Rover.res' using 1:32 title 'Voltage' with line 
set term pop 
replot 
pause -1 
