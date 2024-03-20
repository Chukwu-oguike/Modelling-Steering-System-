%The NLARX model was used to determine the parameters of the system
%{
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

%set sampling rate to that of speed_segment
new_torq = resample(torq_span,speed_sample_fs,torq_sample_fs);
new_gyro = resample(gyro_span,speed_sample_fs,gyro_sample_fs);


%remove bias from data
gyro_bias = 0.09;
torq_bias = 2500;

speed = detrend(speed_span);%detrend before filtering
torq = new_torq - torq_bias ;
gyro = new_gyro - gyro_bias;

%filter data and remove compensate for filtering delay
%I chose FIR filters for linear delay response qualities
b1 = fir1(3000,0.001,'low');
speed = filter(b1,1,speed);
delay1 = mean(grpdelay(b1));
speed(1:delay1) = [];%remove delayed section

b2 = fir1(3000,0.0001,'low');
torq = filter(b2,1,torq);
delay2 = mean(grpdelay(b2));
torq(1:delay2) = [];%remove delayed section

b3 = fir1(3000,0.0001,'low');
gyro = filter(b3,1,gyro);
delay3 = mean(grpdelay(b3));
gyro(1:delay3) = [];%remove delayed section

%match up data points for torq, speed and gyro
speed_size = size(speed);
speed_temp = interp1(1:speed_size, speed, linspace(1, 40000, 40000), 'pchip');

torq_size = size(torq);
torq_temp = interp1(1:torq_size, torq, linspace(1, 40000, 40000), 'pchip');

gyro_size = size(gyro);
gyro_temp = interp1(1:gyro_size, gyro, linspace(1, 40000, 40000), 'pchip');

final_torq = transpose(torq_temp);
final_gyro = transpose(gyro_temp);
final_speed = transpose(speed_temp);

%estimate steering angle

%{
When the radius of curvature of a turn is large compared to the distance
between both wheels the steering angle can be approximated by the equation:

steer_angle = yaw_rate*length_btw_wheels / speed

this condition is true when either yaw rate is very low or velocity is low. 
Examining the plots of yaw_rate and velocity together, we find that either
condition is met for almost every point in the journey.
%}

%Since the length_btw_wheels is constant, we will use points from 
%low resolution data to approximate it. 

length_btw_wheels = 3.94;

%Then use this length to calculatethe new steering angles based on the
%higher resolution yaw_rate and velocity.

%I have added the average speed because I detrended the speed earlier to 
%avoid weird effects from filtering non_zero offset data
speed_ave = mean(speed_span);
final_speed = final_speed + speed_ave;
gyro_bias = 0.09;
final_gyro = final_gyro - gyro_bias;
final_steering = (final_gyro)./final_speed * length_btw_wheels;


Ts = 0.023;

%create iddata object
torq_new_data = iddata(final_steering,final_torq,Ts,'Name', 'Steering_system');
torq_new_data.InputName = 'Voltage';
torq_new_data.InputUnit = 'V';
torq_new_data.OutputName = {'steering angle'};
torq_new_data.OutputUnit = {'rad'};
torq_new_data.Tstart = 0;
torq_new_data.TimeUnit = 's';

opt = nlarxOptions;
opt.Focus = 'simulation';
%}

%The custom regressor is meant to add information about the aligning torq
%to the nonlinear estimator. The aligning torque (which I assume accounts
%for a significant percentage of the nonlinearity) was modelled as a skewed
%sin wave and scaled using the voltage values. More of my thoughts behind
%this approach is explained in the ProjectReport doc.

%System simulation (Takes about 15 to 25 minutes)
sys = nlarx(torq_new_data,[2 1 10],sigmoidnet('num',4), 'customreg',{'Voltage(t-1)+((1/50.0001)*Voltage(t-1).*sin(5*pi*steering angle(t-6)))'},opt);
compare(torq_new_data,sys)%I got a best fit of 65.9% with the measured data.

%linearize model for PID controller
%{
stepinput = iddata([],ones(40000,1),sys.Ts);
[x,u] = findop(sys,'snapshot',2,stepinput);

lnr_sys = linearize(sys,u,x);

%The simulink models for called Controller_linear and Controller_nonLinear
%show the control model I used
%}


