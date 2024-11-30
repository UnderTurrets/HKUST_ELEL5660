function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% TODO: declare global containers for restoring RMS errors.
% global rms_xposition;
% ...;
global rms_xposition;
global rms_xvelocity;
global rms_yposition;
global rms_yvelocity;
global rms_zposition;
global rms_zvelocity;

% TODO: read the current states
% s(1:3) current position (i.e., x, y, z)
% s(4:6) current velocity (i.e., dx, dy, dz)
% s(7:10) current attitude quaternion
% s(11:13) current rate of Euler Angles (i.e., dphi, dtheta, dpsi)
% Note that you don't need to know how does quaternion exactly work here.
% -------------------
% To finish this project, you only need to know how to obtain the rotation 
% matrix from a quaternion, and how to recover the Euler angles from a rotation
% matrix. Two useful functions, i.e., quaternion_to_R.m and RotToRPY_ZXY.m have
% been provided in /utils.
x = s(1:3);
d_x = s(4:6);
quat = s(7:10);
d_angle = s(11:13);

% TODO: read the desired states
% s_des(1:3) desire position (i.e., x_des, y_des, z_des)
% s_des(4:6) desire velocity (i.e., dx_des, dy_des, dz_des)
% s_des(7:10) desire attitude quaternion
% s_des(11:13) desire rate of Euler Angles (i.e., dphi, dtheta, dpsi)
x_c = s_des(1:3);
d_x_c = s_des(4:6);
quat_c = s_des(7:10);
d_angle_c = s_des(11:13);

% TODO: define and set PD-controller parameters
% 位置控制的 PD 控制器参数
kp_xyz = [10.5; 10.5; 10.5];    % Position gain
kd_xyz = [10.5; 10.5; 8.5];    % Velocity gain
kp_angle = [900; 900; 500];  % Attitude gain
kd_angle = [50; 50; 50];  % Angular velocity gain

% TODO: Position control (calculate u1)
% ---------------- 位置控制 ----------------
x_error = x_c - x;
d_x_error = d_x_c - d_x;
dd_x_c = [0;0;0];
dd_x = dd_x_c + kp_xyz .* x_error + kd_xyz .* d_x_error;
F = m * (dd_x(3) + g);

% TODO: Attitude control (calculate u2)
% Rember to deal with angle range or use wrapToPi()
% ---------------- 姿态控制 ----------------
[phi_c,theta_c,psi_c] = RotToRPY_ZXY(QuatToRot(quat_c));
[phi,theta,psi] = RotToRPY_ZXY(QuatToRot(quat));
% refactor phi_c and theta_c
phi_c = 1/g * (dd_x(1) * sin(psi) - dd_x(2) * cos(psi));
theta_c = 1/g * (dd_x(1) * cos(psi) + dd_x(2) * sin(psi));

phi_error = phi_c-phi;
% phi and theta needn't be restricted!!!
% if( ( phi_error >-pi) && ( phi_error <pi) )
% 
% elseif(phi_error<=-pi)
%     phi_error=phi_error + 2 * pi;
% elseif(phi_error>=pi)
%     phi_error=phi_error - 2 * pi;
% end

theta_error = theta_c-theta;
% if( ( theta_error >-pi) && ( theta_error <pi) )
% 
% elseif(theta_error<=-pi)
%     theta_error=theta_error + 2 * pi;
% elseif(theta_error>=pi)
%     theta_error=theta_error - 2 * pi;
% end

psi_error = psi_c-psi;
psi_error = mod(psi_error, 2*pi);
if(psi_error<=-pi)
    psi_error=psi_error + 2 * pi;
elseif(psi_error>=pi)
    psi_error=psi_error - 2 * pi;
end

angle_error = [phi_error;theta_error;psi_error];
d_angle_error = d_angle_c - d_angle;

Rotation = QuatToRot(quat);
dd_angle = kp_angle .* angle_error + kd_angle .* (d_angle_error);
M = I * dd_angle +  cross(Rotation * d_angle, (I * Rotation * d_angle));

% TODO: Record all RMS of states, including position and velocity, for visualization.
% An example is as follows.
% rms_xposition=[rms_xposition,s_des(1)-s(1)];
rms_xposition=[rms_xposition,s_des(1)-s(1)];
rms_xvelocity=[rms_xvelocity,s_des(4)-s(4)];
rms_yposition=[rms_yposition,s_des(2)-s(2)];
rms_yvelocity=[rms_yvelocity,s_des(5)-s(5)];
rms_zposition=[rms_zposition,s_des(3)-s(3)];
rms_zvelocity=[rms_zvelocity,s_des(6)-s(6)];

end