function s_des = trajectory_generator(t, path, h, test_map)
% -------------------------------------------------------------------
% Input:
% t: Query time instance.
% path: A set of waypoints
% h: Handlers for potential visualization.

% Output: 
% s_des: Desired states at query time instance t.

% Function explanation:
% This function contains two parts, implemented respectively
% under "if nargin > 1" and "else".
% The first part implements the trajetory generation given as
% input a set of waypoints.
% The second part returns the desired state at the given query time
% instance.
% -------------------------------------------------------------------

% Some containers for storing data and resulting trajectory coefficients
persistent waypoints traj_time time_interval pos_coeffs vel_coeffs 
s_des = zeros(13,1);
s_des(7:10) = [1;0;0;0];

% Part I
if nargin > 1 % generate a trajectory (given waypoints)
    subplot(h);
    hold on;
    for i = 2: size(test_map, 1) - 1
        cube_x = test_map(i ,1) -1;
        cube_y = test_map(i ,2) -1;
        cube_z = test_map(i ,3) -1;
        color_value = rand;
        plotcube([1,1,1], [cube_x,cube_y,cube_z],1,[color_value,color_value,color_value]);
    end

    waypoints = size(path, 1);
    % TODO: 
    % Assuming the quadrotor flies at unit speed, we calculate
    % each time instance at which the quadrotor passes by each waypoint.
    num_segment = size(path, 1) - 1;
    segment_lengths = sqrt(sum((path(2:end, :) - path(1:end-1, :)).^2, 2)); % 计算每个路径段的长度
    total_length = sum(segment_lengths); % 总长度
    time_interval = 25 .* segment_lengths / total_length; % 每个路径段的时间间隔，单位速度下

    % 计算每个路径段的开始时间
    traj_time = cumsum(time_interval); % 累积每个段的时间
    traj_time = [0; traj_time]; % 在起始位置添加时间0

    % TODO: Minimum-snap trajectory 
    % Prepare the Q matrix for a 7th-order polynomial
    Q = zeros(8 * num_segment, 8 * num_segment);
    Q_block = zeros(8,8);
    for k=1:num_segment
        for i=4:7
            for j=4:7
                Q_block(i+1,j+1)=((i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3))/(i+j-7))*(time_interval(k,1)^(i+j-7));
            end
        end
        Q ((k-1)*8+1:k*8,(k-1)*8+1:k*8) =Q_block;
    end

    % TODO: 
    % Prepare the mapping matrix (polynomial coefficients --> derivatives of states)
    M = zeros(8 * num_segment, 8 * num_segment );
    M_block=zeros(8,8);
    for i = 1:num_segment
        M_block(1,1:end)=[1,0,0,0,0,0,0,0];
        M_block(2,1:end)=[0,1,0,0,0,0,0,0];
        M_block(3,1:end)=[0,0,2,0,0,0,0,0];
        M_block(4,1:end)=[0,0,0,6,0,0,0,0];
        %positon
        M_block(5,1:end)=[1,time_interval(i,1)^1,time_interval(i,1)^2,time_interval(i,1)^3,time_interval(i,1)^4,time_interval(i,1)^5,time_interval(i,1)^6,time_interval(i,1)^7];
        %velocity
        M_block(6,1:end)=[0,1,2*time_interval(i,1)^1,3*time_interval(i,1)^2,4*time_interval(i,1)^3,5*time_interval(i,1)^4,6*time_interval(i,1)^5,7*time_interval(i,1)^6];
        %acceleration
        M_block(7,1:end)=[0,0,2,6*time_interval(i,1)^1,12*time_interval(i,1)^2,20*time_interval(i,1)^3,30*time_interval(i,1)^4,42*time_interval(i,1)^5];
        %jerk
        M_block(8,1:end)=[0,0,0,6,24*time_interval(i,1)^1,60*time_interval(i,1)^2,120*time_interval(i,1)^3,210*time_interval(i,1)^4];
        %assignment
        M((i-1)*8+1:i*8,(i-1)*8+1:i*8) = M_block;
    end

    % TODO:
	% Prepare the selection matrix C.
    C = zeros(8 * num_segment, 8 + 2 * (num_segment - 1) + 3 * (num_segment - 1));
    C(1:4,1:4) = eye(4);
    C(5+8*(num_segment-1):end,5+2*(num_segment-1):8+2*(num_segment-1)) = eye(4);
    if num_segment>1
        for i = 1:(num_segment-1)
            C(8*i-3,2*i+3)=1;
            C(8*i+1,2*i+4)=1;
            for j=1:3       
                C(8*i-3+j,2*num_segment+3*i+3+j)=1;
                C(8*i+1+j,2*num_segment+3*i+3+j)=1;
            end
        end
    end

    % TODO: prepare the R matrix of the unconstained Quadric Programming
    % (QP) problem.
    K_m = M \ C;
    R = K_m' * Q * K_m;
    R_FP = R(1:8+(num_segment-1)*2,8+(num_segment-1)*2+1:end);
    R_PP = R(8+(num_segment-1)*2+1:end,8+(num_segment-1)*2+1:end);

    % TODO: Solve the unconstrained QP problem.
    % Prepare for d_F
    d_Fx = zeros(2 * num_segment + 6,1);
    d_Fx (1,1) = path(1,1);d_Fx (end-3,1) = path(end,1);
    d_Fy = zeros(2 * num_segment + 6,1);
    d_Fy (1,1) = path(1,2);d_Fy (end-3,1) = path(end,2);
    d_Fz = zeros(2 * num_segment + 6,1);
    d_Fz (1,1) = path(1,3);d_Fz (end-3,1) = path(end,3);
    for k=2:waypoints-1
        d_Fx((k-1)*2 -1 + 4 )=path(k,1);
        d_Fx((k-1)*2  + 4 )=path(k,1);
        d_Fy((k-1)*2 -1 + 4 )=path(k,2);
        d_Fy((k-1)*2  + 4 )=path(k,2);
        d_Fz((k-1)*2 -1 + 4 )=path(k,3);
        d_Fz((k-1)*2  + 4 )=path(k,3);
    end

    % TODO: work out d_P
    d_Px_opt = -R_PP \ R_FP' * d_Fx;
    d_Py_opt = -R_PP \ R_FP' * d_Fy;
    d_Pz_opt = -R_PP \ R_FP' * d_Fz;

    % TODO: stack d_F and d_P into d
    d_x = C*[d_Fx;d_Px_opt];
    d_y = C*[d_Fy;d_Py_opt];
    d_z = C*[d_Fz;d_Pz_opt];

    % TODO: mapping back to coefficients of the polynomial (position)
    p_x = M \ d_x;
    p_y = M \ d_y;
    p_z = M \ d_z;
    p_x =reshape(p_x,8,[])';
    p_y =reshape(p_y,8,[])';
    p_z =reshape(p_z,8,[])';

    pos_coeffs = [p_x,p_y,p_z];

    % TODO: work out the coefficients of the velocity polynomial
    v_x = zeros(size(p_x));
    v_y = zeros(size(p_y));
    v_z = zeros(size(p_z));

    for i=1:num_segment
        v_x(i,1:7)=p_x(i,2:8).*[1,2,3,4,5,6,7];
    end
    for i=1:num_segment
        v_y(i,1:7)=p_y(i,2:8).*[1,2,3,4,5,6,7];
    end
    for i=1:num_segment
        v_z(i,1:7)=p_z(i,2:8).*[1,2,3,4,5,6,7];
    end


    vel_coeffs = [v_x v_y v_z];

    % Visualization: plot the trajectory
    subplot(h);
    pos_x = [];
    pos_y = [];
    pos_z = [];
    for i = 1 : num_segment
        T = time_interval(i);
        t = 0:0.01:T;
        pos_xi = polyval(flip(p_x(i,:)),t);
        pos_yi = polyval(flip(p_y(i,:)),t);
        pos_zi = polyval(flip(p_z(i,:)),t); 
        pos_x = [pos_x pos_xi];
        pos_y = [pos_y pos_yi];
        pos_z = [pos_z pos_zi];
    end 
    plot3(pos_x, pos_y, pos_z, 'r-','LineWidth',2);

% Part II: Output desired states at the query time instance
else 
% First, check whether out of range? If so, adjust to the final time.
    if t > traj_time(end)
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1) - 1;  

% Second, deal with the sitation when t == 0
    if t == 0
        s_des(1:3) = path(1,:);
        s_des(7:10) = [1;0;0;0];
% Third, deal with the normal case
    else
        if t_index > 1
            t  = t - traj_time(t_index);
        end

        % load coefficients from the resulting trajectory
        px = pos_coeffs(:,1:8);
        py = pos_coeffs(:,9:16);
        pz = pos_coeffs(:,17:24);

        vx = vel_coeffs(:,1:8);
        vy = vel_coeffs(:,9:16);
        vz = vel_coeffs(:,17:24);

        % Calculate the state at the query time instance t
        pos_x = polyval(flip(px(t_index,:)) , t);
        pos_y = polyval(flip(py(t_index,:)) , t);
        pos_z = polyval(flip(pz(t_index,:)) , t);

        vel_x = polyval(flip(vx(t_index,:)) , t);
        vel_y = polyval(flip(vy(t_index,:)) , t);
        vel_z = polyval(flip(vz(t_index,:)) , t);

        % Output
        s_des(7:10) = [1;0;0;0];
        s_des(1) = pos_x;
        s_des(2) = pos_y;
        s_des(3) = pos_z;

        s_des(4) = vel_x;
        s_des(5) = vel_y;
        s_des(6) = vel_z; 
    end     
end

end


