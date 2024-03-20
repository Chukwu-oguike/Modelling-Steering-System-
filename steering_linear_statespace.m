%This script identifies a linear MISO model of the steering system, with voltage
%across steering column sensor as one input and slip angle as the other
%input and steering angle as the output. 

%In this script the slip angle is defined as a variable that has the same
%profile as the steering angle. However, it is assumed that there is a lag
% and that the rate of change on slip angle is lower than that of the
% steering angle

%This script utilizes a function (steering_column_isee)
%that considers 'steering ratio' as an additional parameter and models
%transmission of aligning to to the rack

load('sample_data_for_system_2018_5_28_isee')

%zero timestamps
speed(:,1) = speed(:,1) - speed(1,1);
torq(:,1) = torq(:,1) - torq(1,1);
gyro(:,1) = gyro(:,1) - gyro(1,1);
steeringcan(:,1) = steeringcan(:,1) - steeringcan(1,1);

%get the average sampling rates
speed_sample_fs = ceil(1/mean(diff(speed(:,1)) ));
gyro_sample_fs = ceil(1/mean(diff(gyro(:,1)) ));
torq_sample_fs = ceil(1/mean(diff(torq(:,1)) ));

%using torq as reference, latency values for sensor in seconds
torq_latency = 0;
speed_latency = 0.45;
steering_latency = 0.49;
gyro_latency = 1;

%latency index

speed_index = ceil(steering_latency * speed_sample_fs);
gyro_index = ceil(gyro_latency * gyro_sample_fs);

%set starting and ending indexes (I ensured that ending indexes were as
%temporally close to each other as possible)
gyro_temp = gyro(:,4);
gyro_span = gyro_temp(gyro_index:5990);

speed_temp = speed(:,2);
speed_span = speed_temp(speed_index:2486);

torq_span = torq(:,2);

new_torq = resample(torq_span,speed_sample_fs,torq_sample_fs);
new_gyro = resample(gyro_span,speed_sample_fs,gyro_sample_fs);

%remove bias from data
gyro_bias = 0.09;
torq_bias = 2500;

%remove bias
new_torq = new_torq - torq_bias ;
new_gyro = new_gyro - gyro_bias;


%match up data points for torq, speed and gyro
speed_size = size(speed_temp);
speed_temp2 = interp1(1:speed_size, speed_temp, linspace(1, 2200, 2200), 'pchip');

torq_size = size(new_torq);
torq_temp2 = interp1(1:torq_size, new_torq, linspace(1, 2200, 2200), 'pchip');

gyro_size = size(new_gyro);
gyro_temp2 = interp1(1:gyro_size, new_gyro, linspace(1, 2200, 2200), 'pchip');

final_torq = transpose(torq_temp2);
final_gyro = transpose(gyro_temp2);
final_speed = transpose(speed_temp2);

%estimate steering angle

%{
When the radius of curvature of a turn is large compared to the distance
between both wheels the steering angle can be approximated by the equation
steer_angle = yaw_rate*length_btw_wheels / speed.
this condition is true when either yaw rate is very low or velocity is low. 
Examining the plots of yaw_rate and velocity together, we find that either
condition is met for almost every point in the journey.
%}

%Since the length_btw_wheels is constant, we will use points from 
%low resolution data to approximate it. 

length_btw_wheels = 2.74;

%Then use this length to calculatethe new steering angles based on the
%higher resolution yaw_rate and velocity.

%I have added the average speed because I detrended the speed earlier to 
%avoid weird effects from filtering non_zero offset data
gyro_bias = 0.09;
final_gyro = final_gyro - gyro_bias;
final_steering = final_gyro./final_speed * length_btw_wheels;

%estimate tire slip angle
%slpAngle = zeros(2200,1);
%volt1 = zeros(2200,1);
slpAngle = final_steering./5;

y = final_steering;
u = horzcat(final_torq,slpAngle);

%create iddata object
torq_new_data = iddata(y, u, 0.023,'Name', 'Steering_system');
torq_new_data.InputName = {'Voltage' 'Slip Angle'};
torq_new_data.InputUnit = {'V' 'rad'};
torq_new_data.OutputName = {'steering angle'};
torq_new_data.OutputUnit = {'rad'};
torq_new_data.Tstart = 0;
torq_new_data.TimeUnit = 's';


%values estimated from google search
Js = 0.04; Bs = 0.072; Ks = 115; Jm = 0.00004; Bm = 0.0032; Mr = 32;
Br = 3820; Kr = 30000; Ki = 0.047; R = 0.37; N = 13.65; Rp = 0.0071; L = 0.0000015; 
Tr = 50; Sr = 14.8;

parameters = {'Steering column inertia',Js;'Steering column viscous damping',Bs;
'Steering column stiffness',Ks;'Motor inertia',Jm;'Motor viscous damping',Bm; 'Rack inertia',Mr; 'rack viscous damping',Br; 
'rack stiffness',Kr;'current constant ',Ki;'terminal resistance ', R;
'motor gear ratio', N;'pinion radius',Rp;'motor inductance',L; 'aligning torque',Tr;...
'steering ratio', Sr;
};

init_sys = idgrey('steering_column_isee',parameters,'c');

compare(torq_new_data,init_sys)



for i = 1:15
    
    init_sys.Structure.Parameters(1, i).Minimum  = 0;
    
end

%{
%set max values for parameters
init_sys.Structure.Parameters(1, 1).Maximum = 0.09;
init_sys.Structure.Parameters(1, 2).Maximum = 0.2;
init_sys.Structure.Parameters(1, 3).Maximum = 200;
init_sys.Structure.Parameters(1, 4).Maximum = 0.00009;
init_sys.Structure.Parameters(1, 5).Maximum = 0.009;
init_sys.Structure.Parameters(1, 6).Maximum = 500;
init_sys.Structure.Parameters(1, 7).Maximum = 40;
init_sys.Structure.Parameters(1, 8).Maximum = 4000;
init_sys.Structure.Parameters(1, 9).Maximum = 30000;
init_sys.Structure.Parameters(1, 10).Maximum = 0.05;
init_sys.Structure.Parameters(1, 11).Maximum = 0.1;
init_sys.Structure.Parameters(1, 12).Maximum = 0.9;
init_sys.Structure.Parameters(1, 13).Maximum = 0.009;
init_sys.Structure.Parameters(1, 14).Maximum = 0.005;
init_sys.Structure.Parameters(1, 15).Maximum = 16;
init_sys.Structure.Parameters(1, 16).Maximum = 1000;
%}
x0spec = idpar('x0',[-0.013; -0.0029; -0.0996; -0.9; 2]);

opt = findstatesOptions;
opt.InitialState = x0spec;
x0 = findstates(init_sys,torq_new_data,Inf,opt);

opt = greyestOptions;
%opt.InitialState = x0;
opt.EnforceStability = true;
opt.DisturbanceMode = 'estimate';
opt.Focus = 'simulation';
opt.SearchMethod = 'Auto';

sys_state_space = greyest(torq_new_data,init_sys,opt);

compare(torq_new_data,sys_state_space)

%}