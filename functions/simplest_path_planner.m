function [ret_val] = simplest_path_planner(q0,dq0,ddq0,q1,dq1, ...
    ddq1, t0, max_joint_velocity, delta_t)
%SIMPLEST_PATH_PLANNER Summary of this function goes here
%   Detailed explanation goes here

num_joints = length(q0);
t1 = max(2*abs((q1-q0)/max_joint_velocity));
t = 0:delta_t:t1;
c = ones(size(t));

A = [
    1   0   0^2     0^3     0^4     0^5;
    0   1   2*0     3*0^2   4*0^3   5*0^4;
    0   0   2       6*0     12*0^2  20*0^3;
    1   t1  t1^2    t1^3    t1^4    t1^5;
    0   1   2*t1    3*t1^2  4*t1^3  5*t1^4;
    0   0   2       6*t1    12*t1^2 20*t1^3;
    ];

qd = zeros(length(t), num_joints);
dqd = zeros(length(t), num_joints);
ddqd = zeros(length(t), num_joints);
for i=1:num_joints
    b = [
        q0(i);
        dq0(i);
        ddq0(i);
        q1(i);
        dq1(i);
        ddq1(i);
        ];
    
    a = A^-1*b;

    qd(:,i) = a(1)*c + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
    dqd(:,i) = a(2)*c + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
    ddqd(:,i) = 2*a(3)*c + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;

end

t = t+t0;

ret_val.qd = qd;
ret_val.dqd = dqd;
ret_val.ddqd = ddqd;
ret_val.t = t;

end

