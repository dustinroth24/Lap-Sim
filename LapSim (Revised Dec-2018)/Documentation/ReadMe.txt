This folder contains all the files necessary to run FSAEM2018_Simp.m, 
which is a MATLAB based lap simulation tool. Vehicle parameters must be put in LapsimDeclerations.m.
The tool is ran by running FSAEM2018_Simp.m. Track info was aquired from Michigan F18 AutoX sketch.SLDPRT,
and compiled in FSAEM2018_autoX.xlsx. The default track is FSAE Mighican 2018 autoX.
Intersections.m is a file pulled from the MATLAB forum. A DOE can be ran by changing the LapSimDOE.m function.
All other functions were written by
Christian Free, Dustin Roth, and Mike Green.


Note: The acceleration solver can be picky. When making a torque curve, be sure the first column is RPM and second column is Nm.
The minimum RPM must translate to bellow a rolling speed of 2.2 m/s. This is how the script takes rollout into acount (and it's not that good of a way).
The minimum RPM must be a factor of 10. The maximum RPM should also be a factor of 10.
