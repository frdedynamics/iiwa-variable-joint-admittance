clear all; clc;

%% Custom ROS messages

if 0 
    % Execute this loop once to create the custom messages.
    % Also READ and follow the 3 Command Window instructions. 
    rosgenmsg("./custom_msg")
end

addpath('./custom_msg\matlab_gen\msggen')

%% Time

% Timestep dt needs to match iiwa expected 
% setSendPeriodMilliSec and setReceiveMultiplier
% e.g. setSendPeriodMilliSec(1), setReceiveMultiplier(2) -> dt = 0.002
dt = 0.002;

load picknplace_qmean.mat

num_samples = length(qmean);
t_start = 0;
t_end = t_start + dt*num_samples;

t = t_start:dt:t_end;
t = t(1:num_samples);


%% Interpolate
multiplier = 10;

if (multiplier > 1)
    qmean_interpolate = interp1(t,qmean,t_start:dt/multiplier:t_end);
    qmean_interpolate = qmean_interpolate(1:multiplier*num_samples-3,:);
    
    t = t_start:dt:multiplier*t_end;
    t = t(1:min(multiplier*num_samples, length(qmean_interpolate)));
    length(t)
    mean_motion.time = t';
    mean_motion.signals.values = qmean_interpolate;
    % mean_motion.signals.dimensions=[DimValues]
   
else
    t = t(1:num_samples);
    length(t)
    mean_motion.time = t';
    mean_motion.signals.values = qmean;
    % mean_motion.signals.dimensions=[DimValues]
end

%% Initial postition
q_0 = mean_motion.signals.values(1,:);
