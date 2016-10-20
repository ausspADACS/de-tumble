clc, clear all
addpath('Orbit3D')
addpath('math')
addpath('igrf')
format bank
disp(['Simulation started at ' datestr(now)])
tic %Initiate runtime counter

%% ***CONSTANTS***
g_parameter = 3.986E14; %graviational parameter of earth, m^3 / s^2
earth_radius = 6378; %radius of earth, km
momInertiaMatrix = [0.0458 0 0; 0 0.0917 0; 0 0 0.1192]; %kg * m^2
conv_to_rad = 0.01745329251;
conv_to_deg = 57.29577951;

%% ***INPUTS***
sim_time = 1440; %minutes in simulation
    %95.63 = 1 orbit
time_step = 10; %seconds between data sample--use numbers below 1
    %DO NOT SET BELOW 0.01 LEST YOU WISH TO MEET THE KRAKEN
k = -75000; %B-dot gain value for de-tumble

% Empty matrices for storing values later
B_field_orb = zeros(sim_time/time_step,3);
B_field_body = zeros(sim_time/time_step,3);
mag_moment = zeros(sim_time/time_step,3); 
torque = zeros(sim_time/time_step,3);
ang_mom = zeros(sim_time/time_step,3);
ang_accel = zeros(sim_time/time_step,3);
meas_quat = zeros(sim_time/time_step,4);

ang_vel(1,:) = [18 -18 18];%Initial angular velocity at deployment, dps
meas_quat(1,:) = [.707 0 0 .707]; %Initial attitude

orb_alt = 550; %mean altitude of orbit, km
inclin = 0; %orbital inclination, degrees
    %0 < inclin < 89 means the satellite rotates with earth
    %91 < inclin < 179 means the satellite rotates against earth
eccent = 0; %eccentricity of orbit, 0 is circular
RA_ascend = 0; %right ascension of ascending node
arg_per   = 0; %argument of perigee
true_anom = 0;  %true anomaly of departure
semi_major = orb_alt + earth_radius; %semi_major axis height, km


%% ***CALCULATE***
orb_vel = sqrt(g_parameter/(semi_major * 1000)); %orbital velocity, km/s
period = (2*pi*(semi_major * 1000)) / orb_vel;
num_orb = (sim_time * 60) / period;
fprintf('Simulating %2.2f Orbits \n', num_orb)
time = datenum(datetime);

% Find the latitude and longitude vectors from orbit specs, starting at 0,0
[lat, long] = Orbit3D(RA_ascend, arg_per, true_anom, inclin, semi_major,...
    eccent, time_step, num_orb);

% Call IGRF to create a matrix of magnetic field values in the NED frame
[Bx_NED, By_NED, Bz_NED] = igrf(time, lat, long, orb_alt);
B_field_NED = [Bx_NED, By_NED, Bz_NED];

% Convert magnetic field vector from NED to orbital frame, row by row
c = 1; %Counter for next for loop for matrix indices
for i = 0:(time_step/60):sim_time
    B_field_orb(c,:) = ned_to_orb(inclin, period, i*60)*B_field_NED(c,:)';
    c = c+1;
end

% Loop for each B field vector, converting to body frame at the start
for j = 1:size(B_field_orb, 1)
    % Convert to unit quaternion for use in rotation combination/vec rotate
    meas_quat(j,:) = meas_quat(j,:)/norm(meas_quat(j,:));
    % Rotate mag vec from orbital to body, convert from nT to T
    B_field_body(j,:) = v_rot_q(B_field_orb(j,:)', meas_quat(j,:)')*10^-9;
    mag_moment(j,:) = cross(B_field_body(j,:),(ang_vel(j,:)*conv_to_rad)); %Teslas X rad/sec
    mag_moment(j,:) = k*mag_moment(j,:);
    torque(j,:) = cross(mag_moment(j,:), B_field_body(j,:)); %N*m
    %ang_mom(j,:) = momInertiaMatrix * ang_vel(j,:)';
    ang_accel(j,:) = (momInertiaMatrix\(torque(j,:)'))'*conv_to_deg;
    % Find change in angle over time step
    angle_change = delta(ang_vel(j,:), ang_accel(j,:), time_step);
    % Convert the change in angle to a quaternion to combine with the 
    % measured quaternion value, then conver to unit quaternion
    change_quat = mat2quat(dcm(angle_change));
    change_quat = change_quat/norm(change_quat);
    % Update angular velocity and attitude, except for the last loop to
    % keep all matrices the same size for plotting against time
    if j < size(B_field_orb, 1)
        meas_quat(j+1,:) = quat_mult2(change_quat, meas_quat(j,:));
        ang_vel(j+1,:) = ang_accel(j,:)*time_step + ang_vel(j,:);
    end
end

%% ***OUTPUT***
% NED frame component graph
total_time = (0:(time_step / 60):sim_time)';
figure()
subplot(2,1,1)
plot(total_time, B_field_NED)
axis([0 sim_time -3.5*10^4 3.5*10^4])
title('Magnetic Field (NED frame)')
legend('Bx', 'By', 'Bz')
xlabel('Time, minutes')
ylabel('nano-Tesla')
% Orbital frame component graph
subplot(2,1,2)
plot(total_time, B_field_orb)
axis([0 sim_time -3.5*10^4 3.5*10^4])
title('Magnetic Field (Orbital Frame)')
legend('Bx', 'By', 'Bz')
xlabel('Time, minutes')
ylabel('nano-Tesla')
% Angular velocity graph
figure()
plot(total_time, ang_vel)
xlim([0 sim_time])
title('Satellite Angular Velocity')
legend('\omegax',  '\omegay', '\omegaz')
xlabel('Time, minutes')
ylabel('Angular Velocity, deg/s')
grid on
% Torque graph
figure()
plot(total_time, torque)
xlim([0 sim_time])
title('Torque Produced')
legend('\taux',  '\tauy', '\tauz')
xlabel('Time, minutes')
ylabel('Torque, N*m')
grid on
% Magnetic Moment graph
figure()
plot(total_time, mag_moment)
xlim([0 sim_time])
title('Magnetic Moments')
legend('\mux',  '\muy', '\muz')
xlabel('Time, minutes')
ylabel('Magnetic Moment, A*m^2')
grid on

% Display sizes to make sure they all match; only for testing code
runtime = toc; %end runtime counter
fprintf('Total Simulation Time = %4.2f \n', runtime)