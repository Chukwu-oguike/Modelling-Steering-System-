%This script identifies a non-linear MISO model of the steering system, with voltage
%across steering column sensor as one input and slip angle as the other
%input and steering angle as the output. 

%In this script, the side slip angle (slip at the center of gravity of the
%vehicle) is used as the tire slip angle because this value can be directly
%estimated from the dat sets available

%This script utilizes a function (steering_col)

%Data preprocessing and Grey-box modelling using a seven-state
%state space model
load('sample_data_for_system_id.mat')

%zero timestamps
speed_segment(:,1) = speed_segment(:,1) - speed_segment(1,1);
torq_segment(:,1) = torq_segment(:,1) - torq_segment(1,1);
gyro_segment(:,1) = gyro_segment(:,1) - gyro_segment(1,1);
steering_can_segment(:,1) = steering_can_segment(:,1) - steering_can_segment(1,1);

%get the average sampling rates
speed_sample_fs = ceil(1/mean(diff(speed_segment(:,1)) ));
gyro_sample_fs = ceil(1/mean(diff(gyro_segment(:,1)) ));
torq_sample_fs = ceil(1/mean(diff(torq_segment(:,1)) ));

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
gyro_temp = gyro_segment(:,4);
gyro_span = gyro_temp(gyro_index:100015);

speed_temp = speed_segment(:,2);
speed_span = speed_temp(speed_index:41486);

torq_temp = torq_segment(:,2);
torq_span = torq_temp(1:10000);

%account for latency between sensors

%remove bias from data
gyro_bias = 0.09;
torq_bias = 2500;

speed = detrend(speed_span);%detrend before filtering
torq = torq_span - torq_bias ;
gyro = gyro_span - gyro_bias;

%
%match up data points for torq, speed and gyro
speed_size = size(speed);
speed_temp = interp1(1:speed_size, speed, linspace(1, 41466, 40000), 'pchip');

torq_size = size(torq);
torq_temp = interp1(1:torq_size, torq, linspace(1, 10000, 40000), 'pchip');

gyro_size = size(gyro);
gyro_temp = interp1(1:gyro_size, gyro, linspace(1, 99915, 40000), 'pchip');

final_torq = transpose(torq_temp);
final_gyro = transpose(gyro_temp);
final_speed = transpose(speed_temp);


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
speed_ave = mean(speed_span);
final_speed = final_speed + speed_ave;
gyro_bias = 0.09;
final_gyro = (final_gyro - gyro_bias)*pi/180;
final_steering = final_gyro./final_speed * length_btw_wheels;

%estimate tire slip angle
Lr_ratio = 0.56;
Lr = length_btw_wheels*Lr_ratio;

%{
for i = 1:2600

     if(final_speed(i) == 0)
         slipYaw = 0;
     else
         slipYaw = asin(final_gyro*Lr./final_speed);
     end
end
%}

slipYaw = asin(final_gyro*Lr./final_speed);

y = final_steering;
u = horzcat(final_torq,slipYaw);

%create iddata object
torq_new_data = iddata(y, u, 0.023,'Name', 'Steering_system');
torq_new_data.InputName = {'Voltage' 'Slip Angle'};
torq_new_data.InputUnit = {'V' 'rad'};
torq_new_data.OutputName = {'steering angle'};
torq_new_data.OutputUnit = {'rad'};
torq_new_data.Tstart = 0;
torq_new_data.TimeUnit = 's';
%}


%
%Initial parameter values
Parameters = [0.04; 0.0225; 172; 0.0000452; 0.0003339; 625; 32;
              3920; 23900; 0.0345; 0.035; 0.5; 0.0071; 9.06e-5];

          
InitialStates = [-1.799; 0; -0.8996; 0; 0.00097; 0; 5];

Order = [1 2 7];

Ts = 0;

init_sys = idnlgrey('steering_col', Order, Parameters, InitialStates, Ts);

%}

compare(torq_new_data,init_sys)


%{
%Ensure that states aren't fixed during simulation
init_sys = setinit(init_sys, 'Fixed', {false false false false false false false});



for i = 1:14
    
    init_sys.Parameters(i).Minimum = 0;
    init_sys.Parameters(i).Fixed = true;
    
end


init_sys.Parameters(10).Fixed = false;
init_sys.Parameters(11).Fixed = false;
init_sys.Parameters(14).Fixed = false;


init_sys.Parameters(1).Maximum = 0.1;
init_sys.Parameters(10).Maximum = 0.1;
init_sys.Parameters(11).Maximum = 0.2;
init_sys.Parameters(14).Maximum = 0.1;

init_sys.SimulationOptions.AbsTol = 1e-9;
init_sys.SimulationOptions.RelTol = 1e-8;


opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchOptions.MaxIterations = 25;
%opt.GradientOptions.Type = 'Refined';
%opt.GradientOptions.MinDifference = 0.001*sqrt(eps);


sys_state_space = nlgreyest(torq_new_data,init_sys,opt);

compare(torq_new_data,sys_state_space)

%}





