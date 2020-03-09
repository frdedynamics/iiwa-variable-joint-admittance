clear all; clc;
fig_num = 1;
max_joint_velocity = 5*pi/180;
max_joint_deccelaration = 15*pi/180;

%% Custom ROS messages

if 0
    % Execute this loop once to create the custom messages.
    % Also READ and follow the 3 Command Window instructions. 
    rosgenmsg("./custom_msg")
end

if ismac || isunix
    addpath("./custom_msg/matlab_gen/msggen")
    addpath("./functions")
elseif ispc
    addpath("./custom_msg\matlab_gen\msggen")
    addpath("./functions")
else
    disp("what kind of wizardry is this machine?")
    return
end

%% ROS master env

if ~contains(getenv('ROS_MASTER_URI'),"http://172.31.1.21:11311")
    user_input = input("Should master be http://172.31.1.21:11311? 1/0");
    if user_input == 1
        setenv('ROS_MASTER_URI','http://172.31.1.21:11311')
    else
        return
    end
end


%% Time

% Timestep dt needs to match iiwa expected 
% setSendPeriodMilliSec and setReceiveMultiplier
% e.g. setSendPeriodMilliSec(1), setReceiveMultiplier(2) -> dt = 0.002
dt = 0.001;

load picknplace2_qmean_qvar.mat
daniel_data.qmean = qmean;
daniel_data.q_var = q_var;
daniel_data.t = t;

num_samples = length(qmean);
t_start = 0;
t_end = t_start + dt*num_samples;

t = t_start:dt:t_end;
t = t(1:num_samples);

figure(fig_num);clf; fig_num = fig_num + 1;
subplot(311);
plot(t,qmean*180/pi, 'LineWidth',1.5);
title('original data')
legend('1','2','3','4','5','6','7')
grid on;
subplot(312);
plot(t(1:end-1),diff(qmean)*180/pi/dt, 'LineWidth',1.5);
grid on;
subplot(313);
plot(t(1:end-2),diff(diff(qmean)/dt)*180/pi/dt, 'LineWidth',1.5);
grid on;

%% Interpolate
multiplier = 3;

if (multiplier > 1)
    for i=1:7
        qmean_interpolate(:,i) = spline(t,qmean(:,i),t_start:dt/multiplier:t_end);
    end
    qmean_interpolate = qmean_interpolate(1:multiplier*num_samples-multiplier,:);
    
    t = t_start:dt:multiplier*t_end;
    t = t(1:min(multiplier*num_samples, length(qmean_interpolate)));
%     length(t)
    mean_motion.time = t';
    mean_motion.signals.values = qmean_interpolate;
    % mean_motion.signals.dimensions=[DimValues]
   
else
    t = t(1:num_samples);
    mean_motion.time = t';
    mean_motion.signals.values = qmean;
    % mean_motion.signals.dimensions=[DimValues]
end

clear qmean_interpolate multiplier qmean

%% Initial postition
q_0 = mean_motion.signals.values(1,:);

%% Plot 
figure(99); clf;
    
subplot(311)
plot(mean_motion.time, mean_motion.signals.values*180/pi, 'LineWidth',1.5);
hold on
grid on

subplot(312)
plot(mean_motion.time(1:end-1),(180/pi)*diff(mean_motion.signals.values)/dt, 'LineWidth',1.5)
hold on
grid on

subplot(313)
plot(mean_motion.time(1:end-2),(180/pi)*diff(diff(mean_motion.signals.values)/dt)/dt, 'LineWidth',1.5)
hold on
grid on

%% Slow down
q_end_motion = mean_motion.signals.values(end,:);
tmp = diff(mean_motion.signals.values)/dt;
dq_end_motion = tmp(end,:);
tmp = diff(tmp)/dt;
ddq_end_motion = tmp(end,:);
clear tmp
if any(abs(dq_end_motion) > max_joint_velocity)
    
    
    q_end_motion = mean_motion.signals.values(end,:);
    tmp = diff(mean_motion.signals.values)/dt;
    dq_end_motion = tmp(end,:);
    tmp = diff(tmp)/dt;
    ddq_end_motion = tmp(end,:);
    tmp = diff(tmp)/dt;
    dddq_end_motion = tmp(end,:);

    tmp = simplest_path_planner(dq_end_motion, ddq_end_motion,dddq_end_motion, ...
        zeros(size(q_0)), zeros(size(q_0)), zeros(size(q_0)), ...
        mean_motion.time(end), max_joint_deccelaration, dt);
    tmp.ddqd = tmp.dqd;
    tmp.dqd = tmp.qd;
    tmp.qd = q_end_motion + cumsum(tmp.dqd*dt);
    
    figure(99);
    subplot(311)
    plot(tmp.t, tmp.qd*180/pi,'k--', 'LineWidth',1.5)
    subplot(312)
    plot(tmp.t(1:end-1), tmp.dqd(1:end-1,:)*180/pi,'k--', 'LineWidth',1.5)
    subplot(313)
    plot(tmp.t(1:end-2), tmp.ddqd(1:end-2,:)*180/pi,'k--', 'LineWidth',1.5)
    
    mean_motion.time = [mean_motion.time; tmp.t'];
    mean_motion.signals.values = [mean_motion.signals.values; tmp.qd];
    
    figure(fig_num); clf; fig_num = fig_num + 1;
    subplot(311);
    plot(tmp.t,tmp.qd*180/pi)
    grid on
    title('Slow down')
    subplot(312);
    plot(tmp.t(1:end-1),tmp.dqd(1:end-1,:)*180/pi)
    hold on
    grid on
    plot([tmp.t(1), tmp.t(end)], [max_joint_velocity, max_joint_velocity]*180/pi, 'r--')
    plot([tmp.t(1), tmp.t(end)], [-max_joint_velocity, -max_joint_velocity]*180/pi, 'r--')
    subplot(313);
    plot(tmp.t(1:end-2),tmp.ddqd(1:end-2,:)*180/pi)
    grid on
end


clear q_end_motion dq_end_motion ddq_end_motion

%% Back to start

figure(99);

q_end_motion = mean_motion.signals.values(end,:);
tmp = diff(mean_motion.signals.values)/dt;
dq_end_motion = tmp(end,:);
tmp = diff(tmp)/dt;
ddq_end_motion = tmp(end,:);
% dq_end_motion = (mean_motion.signals.values(end,:)-...
%     mean_motion.signals.values(end-1,:))/dt
% ddq_end_motion = (dq_end_motion-((mean_motion.signals.values(end-1,:)-...
%     mean_motion.signals.values(end-2,:))/dt))/dt

tmp = simplest_path_planner(q_end_motion, dq_end_motion, ddq_end_motion, ...
    q_0, zeros(size(q_0)), zeros(size(q_0)), ...
    mean_motion.time(end), max_joint_velocity, dt);


mean_motion.time = [mean_motion.time; tmp.t'];
mean_motion.signals.values = [mean_motion.signals.values; tmp.qd];

subplot(311)
plot(tmp.t, tmp.qd*180/pi, 'LineWidth',1.5)
plot(tmp.t, tmp.qd*180/pi,'k--', 'LineWidth',1)
legend('1','2','3','4','5','6','7')

subplot(312)
plot(tmp.t(1:end-1), tmp.dqd(1:end-1,:)*180/pi, 'LineWidth',1.5)
plot(tmp.t(1:end-1), tmp.dqd(1:end-1,:)*180/pi,'k--', 'LineWidth',1)

subplot(313)
plot(tmp.t(1:end-2), tmp.ddqd(1:end-2,:)*180/pi, 'LineWidth',1.5)
plot(tmp.t(1:end-2), tmp.ddqd(1:end-2,:)*180/pi,'k--', 'LineWidth',1)

clear tmp q_end_motion dq_end_motion ddq_end_motion

%% Run a few rounds

mean_motion.time = [
    mean_motion.time;
    mean_motion.time+mean_motion.time(end) + dt;
    mean_motion.time+2*mean_motion.time(end) + 2*dt;
    mean_motion.time+3*mean_motion.time(end) + 3*dt
    ];
mean_motion.signals.values = [
    mean_motion.signals.values; 
    mean_motion.signals.values;
    mean_motion.signals.values;
    mean_motion.signals.values
    ];

figure(fig_num);clf; fig_num = fig_num + 1;
plot(mean_motion.time, mean_motion.signals.values*180/pi)
legend('1','2','3','4','5','6','7')
hold on
grid on

%% Update simulink time
t = mean_motion.time;

figure(99);
subplot(311)
axis tight
subplot(312)
axis tight
subplot(313)
axis tight


