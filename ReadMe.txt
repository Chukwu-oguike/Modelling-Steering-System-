1) steering_sys_nlarx.m This script creates a model of the steering system of an autonomous car using the combination of a linear
polynomial model and sigmoidal neural network. 

steering_linear_statespace.m: This script identifies a linear MISO statespace model of the steering system, with voltage
across steering column sensor as one input and slip angle as the other
input and steering angle as the output.

In this script the slip angle is defined as a variable that has the same
profile as the steering angle. However, it is assumed that there is a lag
and that the rate of change on slip angle is lower than that of the
steering angle.

This script utilizes a function (steering_column_isee)
that considers 'steering ratio' as an additional parameter and models
transmission of aligning to to the rack.

2) steering_nl_statespace.m: Data preprocessing and nonlinear grey-box modelling using a seven-state
state space model and using slip angle as one of two inputs to the system 
(aligning torque is a function of slip angle). In this script I used a kinematic 
model to estimate the slip angle at the center of gravity of the vehicle.

3) steering_tf_sys.m: This script creates a stack of LTI, MISO black-box models of the steering system using 
velocity and voltage applied to the torque sensor as inputs and the steering angle as outputs.

4) steering_col.m: This is the the seven-state linear state space model (although I used non-linear grey-box identification)
of the steering system which comprises of the power steering motor, rack, pinion and steering column. I assumed
that Driver torq is zero.

5) steering_column_isee.m: This is the the five-state linear state space model of a COMLUMN-TYPE steering system which comprises of
the power steering motor, rack, pinion and steering column. I assumed that Driver torq is zero.



Note: the theory behind the statespace models and nlarx model is explained the the "Project Report" doc.