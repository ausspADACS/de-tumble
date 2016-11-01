% aerodynamic torque at a given point

function torque = aero_torque(density, velocity, attitude);
% geometric center of satellite is taken as (0, 0, 0)
% center of mass
x_com = -16E-3; %m
y_com = 1.26E-3; %m
z_com = -1.79E-3; %m

%fin lengths
large_fin_l = 318.5E-3; %m
side_fin_l = 318.5E-3; %m

%area of satellite
top_panel = (353E-3 * 226.3E-3); %m^2
side_panel = (353E-3 * 100E-3); %m^2
large_fin = (207.3E-3 * 318.5E-3); %m^2
side_fin = (318.5E-3 * 81E-3); %m^2

density = 3.365E-13; %kg/m^2
velocity = 7.58516E3; %m/s

fin_angle = 160; %degrees
theta = 160 - 90; %degrees

% convert attitude to yaw, pitch, roll

  % *****PITCH*****
    %body
    Fbody_pitch = [0.5 * density * velocity^2 * top_panel, 0]; %N
    Fb_location_body_pitch = [0, 0];
    Fb_distance_body_pitch = Fb_location_body_pitch - [x_com, z_com];
    tau_pitch_body_pitch = Fbody_pitch .* Fb_distance_body_pitch;
    
    %fins
    Ffins_pitch = 0.5 * density * (velocity * sind(theta))^2 * large_fin; %N
    Fb_location_fin_pitch = [-50E-3 + (0.5 * cosd(theta) * large_fin_l), -176.5E-3 + (0.5 * sind(theta) * large_fin_l)];
    Fb_distance_fin_pitch = Fb_location_fin_pitch - [x_com, z_com];
    tau_pitch_fin_pitch = Ffins_pitch .* Fb_distance_fin_pitch;
    
    % calculate torque, pitch
    tau_pitch = norm(tau_pitch_body_pitch + tau_pitch_fin_pitch);
  
  % *****YAW*****
    %body
    Fbody_yaw = [0.5 * density * velocity^2 * side_panel, 0]; %N
    Fb_location_body_yaw = [0, 0];
    Fb_distance_body_yaw = Fb_location_body_yaw - [y_com, x_com];
    tau_yaw_body_yaw = Fbody_yaw .* Fb_distance_body_yaw;
    
    %fins
    Ffins_yaw = 0.5 * density * (velocity * sind(theta))^2 * side_fin; %N
    Fb_location_fin_yaw = [-113.15E-3 + (0.5 * cosd(theta) * side_fin_l), -176.5E-3 + (0.5 * sind(theta) * side_fin_l)];
    Fb_distance_fin_yaw = Fb_location_fin_yaw - [y_com, x_com];
    tau_yaw_fin_yaw = Ffins_yaw .* Fb_distance_fin_yaw;
    
  % calculate torque, yaw
  tau_yaw = norm(tau_yaw_body_yaw + tau_yaw_fin_yaw);
  
  torque_max = [tau_pitch, tau_yaw];
  
  torque_pitch = torque_max * sind(angle_of_attack);
  torque_yaw = torque_max * sind(angle_of_yaw);
  torque = [torque_pitch, torque_yaw];

end