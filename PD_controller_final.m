clc, clear
format short
tic

%% CONSTANTS
for i=1:1
des_quat = [1 0 0 0]; %desired quaternion
dt = 0.01; %timestep
TIME = 10; %total simulation time, seconds
Kp =    200;  %proportional
Kd =   -100;  %derivative
ang_vel = [.5 1 .7]'; %initial angular velocity measurement, rad/s
end

%% VARIABLES
for i=1:1
init_quat = [sqrt(2)/2 -sqrt(2)/2 0 0]'; %initial measured quaternion
    meas_quat = init_quat; %the initial quaternion acts as the first measured quaternion

quat_err = quatRelate(des_quat, meas_quat); %initial error based on desired/measured quaternion

ang_acc = Kp * quat_err(2:4) + Kd * ang_vel; %necessary angular acceleration
ang_vel = ang_vel + ang_acc*dt; %angular velocity recalculated based on acceleration

qdot = quatMultiply(ang_vel, meas_quat); %qdot calculated by quaternion multiplication

sat_mass = 10; % kg
sat_dims = [.353 .2263 .1]; % x,y,z dimensions of  cube in meters
sat_moi = (1/12*sat_mass)*[(sat_dims(2)^2+sat_dims(3)^2)  0  0; %satellite moment of inertia
                           0 (sat_dims(1)^2+sat_dims(3)^2) 0;
                           0 0 (sat_dims(1)^2+sat_dims(2)^2)];
wheel_moi = [.00000523 0 0; 0 .00000523 0; 0 0 .00000523]; %reaction wheel moment of inertia

wheel_vel = [0 0 0]'; %initial reaction wheel velocity is 0

wheel_acc = (cross(((sat_moi*ang_vel) + wheel_moi*wheel_vel), ang_vel) ...
    - sat_moi*ang_acc)/wheel_moi(1); %calculate wheel acceleration

wheel_vel = wheel_vel + wheel_acc*dt; %recalculate wheel velocity after acceleration

meas_quat = qdot*dt + init_quat; %measured quaternion recalculated before loop
end

%% CALCULATE
for i=1:1
quat_err_list = quat_err; 
ang_acc_list = ang_acc; ang_vel_list = ang_vel;
wheel_acc_list = wheel_acc; wheel_vel_list = wheel_vel;
T = 1;
Tlist = T;
while T < TIME
    %PID Controller
        ang_acc = Kp*quat_err(2:4) + Kd*ang_vel; %angular acceleration based on PID components

        ang_vel = ang_vel + ang_acc*dt; %updates the angular velocity with new acceleration

        ang_vel_list = [ang_vel_list, ang_vel + ang_acc*dt]; %create list of angular velocities
        
        qdot = quatMultiply(ang_vel, meas_quat); %recalculate qdot with quaternion multiplication

        meas_quat = meas_quat + qdot*dt; %measure new quaternion using qdot
        quat_err = quatRelate(des_quat, meas_quat); %find new error based on current measured quaternion

        quat_err_list = [quat_err_list, quat_err]; %accumulate error into list
        ang_acc_list = [ang_acc_list, ang_acc]; %accumulate acceleration into list
        
    
    %Reaction wheel
        wheel_acc = (cross(((sat_moi*ang_vel) + wheel_moi*wheel_vel), ...
            ang_vel) - sat_moi*ang_acc)/wheel_moi(1);
            %wheel acceleration is based on the satellite and wheel moments of
            %inertia, as well as satellite angular velocity and acceleration

        wheel_vel = wheel_vel + wheel_acc*dt; %reaction wheel velocity

        wheel_acc_list = [wheel_acc_list, wheel_acc]; %create a list of accelerations
        wheel_vel_list = [wheel_vel_list, wheel_vel]; %create a list of velocities
        
    
    T = T + dt; %timestep of dt
    Tlist = [Tlist, T];
end
end

%% DISPLAY
for i=1:1
    figure()
    set(gcf,'color','w');
    plot(Tlist, quat_err_list')
    xlabel('Time (Seconds)')
    ylabel('Quaternion Error')
    legend('q0', 'q1', 'q2', 'q3')
    title('Quaternion Error Over Time')
    %{
    figure()
    set(gcf,'color','w');
    plot(Tlist, wheel_acc_list')
    xlabel('seconds')
    ylabel('rad/s^2')
    legend('Accel.')
    title('Wheel Acceleration Over Time')
    %}
    figure()
    set(gcf,'color','w');
    plot(Tlist, wheel_vel_list')
    xlabel('Time (Seconds)')
    ylabel('rpm')
    legend('\omegax', '\omegay', '\omegaz')
    title('Reaction Wheel Angular Velocity Over Time')
    %{
    figure()
    set(gcf,'color','w');
    plot(Tlist, ang_acc_list')
    xlabel('seconds')
    ylabel('rad/s^2')
    legend('Accel.')
    title('Satellite Angular Acceleration')
    %}
    figure()
    set(gcf,'color','w');
    plot(Tlist, ang_vel_list')
    xlabel('Time (Seconds)')
    ylabel('rad/s')
    legend('\omegax', '\omegay', '\omegaz')
    title('Satellite Angular Velocity Over Time')
end

%%

run_time = toc