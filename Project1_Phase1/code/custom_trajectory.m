function s_des = custom_trajectory(t, true_s)
    
    s_des = zeros(13, 1);

    % 定义参数
    side_length = 4;  % 正方形的边长
    period = 20;       % 完成一个循环的时间
    t1 = mod(t, period);  % 将 t 映射到 [0, period] 区间

    % 计算在正方形的哪个边
    if t1 <= period / 4
        % 第一边 (右)
        y_des = side_length / 2 - side_length * (t1 / (period / 4));
        x_des = side_length / 2;
        y_vdes = -side_length / (period / 4);
        x_vdes = 0;
        des_yaw = -pi/2;
    elseif t1 <= period / 2
        % 第二边 (下)
        y_des = -side_length / 2;
        x_des = side_length / 2 - side_length * ((t1 - period / 4) / (period / 4));
        y_vdes = 0;
        x_vdes = -side_length  / (period / 4);
        des_yaw = -pi;
    elseif t1 <= 3 * period / 4
        % 第三边 (左)
        y_des = -side_length / 2 + side_length * ((t1 - period / 2) / (period / 4));
        x_des = -side_length / 2;
        y_vdes = side_length  / (period / 4);
        x_vdes = 0;
        des_yaw = - 1.5 *pi;
    else
        % 第四边 (上)
        y_des = side_length / 2;
        x_des = -side_length / 2 + side_length * ((t1 - 3 * period / 4) / (period / 4));
        y_vdes = 0;
        x_vdes = side_length / (period / 4);
        des_yaw = -2 * pi;
    end

    % Z轴线性变化
    z_des = 3 / 25 * t;  % Z轴线性变化
    z_vdes = 3 / 25;  % dz/dt

    % 填充状态向量
    s_des(1) = x_des;
    s_des(2) = y_des;
    s_des(3) = z_des;
    s_des(4) = x_vdes;
    s_des(5) = y_vdes;
    s_des(6) = z_vdes;

    % 期望偏航角
    des_yaw = mod(0.1 * pi * t, 2 * pi);
    ypr = [des_yaw, 0.0, 0.0];
    Rot = ypr_to_R(ypr);
    q_des = R_to_quaternion(Rot);
    s_des(7:10) = q_des;
end
