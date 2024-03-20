%this script creates LTI, MISO black-box models of the steering system using 
%velocity and voltage applied to the torque sensor as inputs and the
%steering angle as outputs
%
load('throttle_system_valid_data_2_2018-06-17_54.mat')

%
speed(:,1) = speed(:,1) - speed(1,1);
torq(:,1) = torq(:,1) - torq(1,1);
gyro(:,1) = gyro(:,1) - gyro(1,1);
steeringcan(:,1) = steeringcan(:,1) - steeringcan(1,1);
steeringimu(:,1) = steeringimu(:,1) - steeringimu(1,1);
imuacchistory(:,1) = imuacchistory(:,1) - imuacchistory(1,1);

%get the average sampling rates
speed_sample_fs = ceil(1/mean(diff(speed(:,1)) ));

%using torq as reference, latency values for sensor in seconds
speed_latency = 0.45;

%latency index

speed_index = ceil(speed_latency * speed_sample_fs);

%set starting and ending indexes (I ensured that ending indexes were as
%temporally close to each other as possible)
speed_temp = speed(:,2);
speed_span = speed_temp(speed_index:2486);

torq_temp = torq(:,2);
torq_span = torq_temp(1:600);

gyro_temp = gyro(:,4);
gyro_span = gyro_temp(1:5991);

steeringcan_temp = steeringcan(:,2);
steeringcan_span = steeringcan_temp(1:4970);

steeringimu_temp = steeringimu(:,2);
steeringimu_span = steeringimu_temp(1:5990);

imuaccX_temp = imuacchistory(:,2);
imuaccX_span = imuaccX_temp(1:5991);

imuaccY_temp = imuacchistory(:,3);
imuaccY_span = imuaccY_temp(1:5991);

%remove torq sensor bias
torq_bias = 2500;
torq_span = torq_span - torq_bias ;

%match up data points for torq, speed and gyro
speed_size = size(speed_span);
speed_temp2 = interp1(1:speed_size, speed_span, linspace(1,2468 , 2600), 'pchip');

steeringcan_size = size(steeringcan_span);
steeringcan_temp2 = interp1(1:steeringcan_size, steeringcan_span, linspace(1,4970 , 2600), 'pchip');

steeringimu_size = size(steeringimu_span);
steeringimu_temp2 = interp1(1:steeringimu_size, steeringimu_span, linspace(1,5990 , 2600), 'pchip');

imuaccX_size = size(imuaccX_span);
imuaccX_temp2 = interp1(1:imuaccX_size, imuaccX_span, linspace(1,5991 , 2600), 'pchip');

imuaccY_size = size(imuaccY_span);
imuaccY_temp2 = interp1(1:imuaccY_size, imuaccY_span, linspace(1,5991 , 2600), 'pchip');

torq_size = size(torq_span);
torq_temp2 = interp1(1:torq_size, torq_span, linspace(1, 600, 2600), 'pchip');

gyro_size = size(gyro_span);
gyro_temp2 = interp1(1:gyro_size, gyro_span, linspace(1, 5436, 2600), 'pchip');

final_speed = transpose(speed_temp2);
final_torq = transpose(torq_temp2);
final_gyro = transpose(gyro_temp2);
final_steeringcan = transpose(steeringcan_temp2);
final_steeringimu = transpose(steeringimu_temp2);
final_imuaccX = transpose(imuaccX_temp2);
final_imuaccY = transpose(imuaccY_temp2);

%{
curb_length = 2.74;
Lr_ratio = 0.2;
Lr = curb_length*Lr_ratio;

slipYaw = asin(final_gyro*Lr./final_speed);
slipSteering = atan(Lr_ratio*tan(final_steeringimu));

plot(slipYaw,'r')
hold on
plot(slipSteering,'b')
plot(final_steeringimu,'g')

%}


%
y = final_steeringimu;
%final_torq = zeros(2600,1);
u = horzcat(final_torq, final_speed);

%
%create iddata object
torq_new_data = iddata(y, u, 0.023,'Name', 'steering_system');
torq_new_data.InputName = {'EPS Voltage' 'slip'};
torq_new_data.InputUnit = {'V' 'rads'};
torq_new_data.OutputName = {'Steering Angle'};
torq_new_data.OutputUnit = {'rads'};
torq_new_data.Tstart = 0;
torq_new_data.TimeUnit = 's';


%Loop creates varying orders of transfer function models of steering system
%with torque as input and then validates these models with data set.
p2 = 4;
z2 = 1;
n = 1:10;
models = cell(1,45);
ct = 1;
for i = 2:10
    p1 = n(i);
    for j = 1:i-1
        z1 = n(j);
                models{ct} = tfest(torq_new_data,[p1 p2],[z1 z2]);
                ct = ct+1;            
    end
end
%

%model validation
models = stack(1,models{:});
compare(torq_new_data,models)
%}