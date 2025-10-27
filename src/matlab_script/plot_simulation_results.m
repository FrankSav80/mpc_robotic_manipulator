%% MATLAB script to plot observer and robot tracking data
clear; clc; close all;

%% 1. Settings
% Log file to analyze
% log_file_path = 'C:/Users/pc/Desktop/Task3/CBFSC/Sim/Prova 1/Sim_A.txt';
% log_file_path = 'C:/Users/pc/Desktop/Task3/CBFSC/Sim/3. Tuning R/Sim1__N_2__P_gamma_25__R=5.txt';
% log_file_path = 'C:/Users/pc/Desktop/debug quaternione/New 2.0/1. Tuning R/1. N=5 R=50 Q_pos=2000 Q_orient=10/4.txt';
log_file_path = 'C:/Users/pc/Desktop/debug quaternione/New 2.0/2. Tuning Q_orient/2. N=5 R=2 Q_pos=2000 Q_orient=50/o.txt';
dt_plot = 0.1; % MPC dt, for the time axis in seconds

%% 2. Data structure initialization
log_data = struct();
log_data.t_idx = [];
log_data.o_measured = {}; log_data.o_estimated = {}; log_data.v_estimated = {};
log_data.a_estimated = {}; log_data.o_predicted_step1 = {};
log_data.pos_ref = {}; log_data.quat_ref = {}; log_data.pos_log = {}; log_data.quat_log = {};
log_data.u_log = {}; 
log_data.gamma_log = {};

%% 3. Reading, parsing and data conversion
fid = fopen(log_file_path, 'r');
if fid == -1, error('Error: Unable to open log file: %s', log_file_path); return; end
fprintf('Reading log file: %s\n', log_file_path);
line_num = 0; current_t_from_log = -1;

while ~feof(fid)
    log_line = fgetl(fid); line_num = line_num + 1;

    if isempty(log_line), continue; end
    new_t_tokens = regexp(log_line, 't=(\d+)', 'tokens');
    if ~isempty(new_t_tokens) && ~isempty(new_t_tokens{1}), current_t_from_log = str2double(new_t_tokens{1}{1}); end
    if current_t_from_log < 0, continue; end
    mat_idx = current_t_from_log + 1;

    if mat_idx > length(log_data.t_idx)
        log_data.t_idx(mat_idx) = current_t_from_log;
        log_data.o_measured{mat_idx} = [NaN NaN NaN]; log_data.o_estimated{mat_idx} = [NaN NaN NaN];
        log_data.v_estimated{mat_idx} = [NaN NaN NaN]; log_data.a_estimated{mat_idx} = [NaN NaN NaN];
        log_data.o_predicted_step1{mat_idx} = [NaN NaN NaN]; log_data.pos_ref{mat_idx} = [NaN NaN NaN];
        log_data.quat_ref{mat_idx} = [NaN NaN NaN NaN]; log_data.pos_log{mat_idx} = [NaN NaN NaN];
        log_data.quat_log{mat_idx} = [NaN NaN NaN NaN];
        log_data.u_log{mat_idx} = [NaN NaN NaN NaN NaN NaN];
        log_data.gamma_log{mat_idx} = NaN;
    end

    numeric_tokens_4d = regexp(log_line, '\[\s*(-?[\d\.]+),\s*(-?[\d\.]+),\s*(-?[\d\.]+),\s*(-?[\d\.]+)\s*\]', 'tokens');
    numeric_tokens_3d = regexp(log_line, '\[\s*(-?[\d\.]+),\s*(-?[\d\.]+),\s*(-?[\d\.]+)\s*\]', 'tokens');
    
    if ~isempty(numeric_tokens_4d) && length(numeric_tokens_4d{1}) == 4
        data_vector = cellfun(@str2double, numeric_tokens_4d{1});
        if contains(log_line, "q_ref"), log_data.quat_ref{mat_idx} = data_vector;
        elseif contains(log_line, "q_pred"), log_data.quat_log{mat_idx} = data_vector; end
    elseif ~isempty(numeric_tokens_3d) && length(numeric_tokens_3d{1}) == 3
        data_vector = cellfun(@str2double, numeric_tokens_3d{1});
        if contains(log_line, "o_misurato_usato_da_obs"), log_data.o_measured{mat_idx} = data_vector;
        elseif contains(log_line, "o_stimato_attuale"), log_data.o_estimated{mat_idx} = data_vector;
        elseif contains(log_line, "v_stimato_attuale"), log_data.v_estimated{mat_idx} = data_vector;
        elseif contains(log_line, "a_stimato_attuale"), log_data.a_estimated{mat_idx} = data_vector;
        elseif contains(log_line, "o_pred_orizzonte[1]"), log_data.o_predicted_step1{mat_idx} = data_vector;
        elseif contains(log_line, "p_ref"), log_data.pos_ref{mat_idx} = data_vector;
        elseif contains(log_line, "p_pred"), log_data.pos_log{mat_idx} = data_vector; end
    else
        % Parsing for control commands u[i]
        tokens_u = regexp(log_line, 'u\[(\d)\]:\s*(-?[\d\.]+)','tokens');
        if ~isempty(tokens_u)
            joint_idx = str2double(tokens_u{1}{1}) + 1;
            u_val = str2double(tokens_u{1}{2});
            log_data.u_log{mat_idx}(joint_idx) = u_val;
        end
        % Parsing for gamma
        tokens_gamma = regexp(log_line, 'gamma_ee_optimale\[0\].*:\s*(-?[\d\.]+)','tokens');
        if ~isempty(tokens_gamma)
            log_data.gamma_log{mat_idx} = str2double(tokens_gamma{1}{1});
        end
    end
end
fclose(fid);
fprintf('End of log file reading.\n');


% --- Start conversion and data preparation ---
last_valid_idx = current_t_from_log + 1;
if last_valid_idx <= 0, fprintf('No valid data found in the log file.\n'); return; end
time_vector = log_data.t_idx(1:last_valid_idx)';
o_measured_data = cell2mat(log_data.o_measured(1:last_valid_idx)');
o_estimated_data = cell2mat(log_data.o_estimated(1:last_valid_idx)');
v_estimated_data = cell2mat(log_data.v_estimated(1:last_valid_idx)');
a_estimated_data = cell2mat(log_data.a_estimated(1:last_valid_idx)');
o_predicted_step1_data = cell2mat(log_data.o_predicted_step1(1:last_valid_idx)');
pos_ref_data = cell2mat(log_data.pos_ref(1:last_valid_idx)');
quat_ref_data = cell2mat(log_data.quat_ref(1:last_valid_idx)');
pos_log_data = cell2mat(log_data.pos_log(1:last_valid_idx)');
quat_log_data = cell2mat(log_data.quat_log(1:last_valid_idx)');
u_log_data = cell2mat(log_data.u_log(1:last_valid_idx)');
gamma_log_data = cell2mat(log_data.gamma_log(1:last_valid_idx)');

num_timesteps = size(quat_ref_data, 1);

% Align log quaternions to the reference (done only once)
quat_log_aligned = quat_log_data;
for k = 1:num_timesteps
    if dot(quat_ref_data(k,:), quat_log_aligned(k,:)) < 0
        quat_log_aligned(k,:) = -quat_log_aligned(k,:);
    end
end

% Compute all error metrics
angle_err_rad_vec = zeros(num_timesteps, 1);
err_orient_vec_mat = zeros(num_timesteps, 3);
axis_err_mat = zeros(num_timesteps, 3);

for i = 1:num_timesteps
    q_ref = quat_ref_data(i, :)';
    q_pred = quat_log_aligned(i, :)';

    q_pred_conj = quat_conj_matlab(q_pred);
    q_err = quat_mult_matlab(q_ref, q_pred_conj);

    err_orient_vec_mat(i, :) = (2 * q_err(1:3))';

    qv = q_err(1:3);
    qw = q_err(4);
    qv_norm = norm(qv);
    angle_err_rad_vec(i) = 2 * atan2(qv_norm, qw);

    if qv_norm > 1e-9 % Avoid division by zero
        axis_err = qv / qv_norm;
    else
        axis_err = [0; 0; 0]; % Axis not defined if angle is zero
    end
    axis_err_mat(i, :) = axis_err'; % Store the axis
end

algebraic_quat_error_mat = quat_ref_data - quat_log_aligned;
err_position = pos_ref_data - pos_log_data;

%% 4. Data plotting
time_seconds = time_vector * dt_plot;
fine_reference_time = 30 * dt_plot;
comp_names = {'X', 'Y', 'Z'};
quat_comp_names = {'q_x', 'q_y', 'q_z', 'q_w'};

%% --- Plot 1-2: Tracking performance of position and orientation (EE) ---
% --- Plot 1: Tracking performance of EE position ---
figure('Name', 'Position tracking');
for i = 1:3
    subplot(3, 1, i);
    plot(time_seconds, pos_ref_data(:, i), 'k:', 'LineWidth', 2, 'DisplayName', 'Desired position');
    hold on;
    plot(time_seconds, pos_log_data(:, i), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Measured position');
    xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
    hold off;
    xlabel('Time (s)'); ylabel(['Position ' comp_names{i} ' (m)']);
    title(['Robot position tracking - component ' comp_names{i}]);
    legend show; grid on;
end
sgtitle('Robot end-effector position tracking performance');

% --- Plot 2: Tracking performance of EE orientation ---
figure('Name', 'Orientation tracking (quaternion components)');
for i = 1:4
    subplot(4, 1, i);
    plot(time_seconds, quat_ref_data(:, i), 'k:', 'LineWidth', 2, 'DisplayName', 'Desired orientation');
    hold on;
    plot(time_seconds, quat_log_aligned(:, i), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Measured orientation');
    xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
    hold off;
    xlabel('Time (s)'); ylabel(['Component ' quat_comp_names{i}]);
    title(['Robot orientation tracking - component ' quat_comp_names{i}]);
    legend show; grid on;
end
sgtitle('Robot orientation tracking performance (quaternion components)');

%% --- Plot 3: EE position error analysis --- 
colors = {'r', 'g', 'b'}; 

for i = 1:3
    subplot(3, 1, i);
    plot(time_seconds, err_position(:, i), colors{i}, 'LineWidth', 1.5, 'DisplayName', ['Error ' comp_names{i}]);
    hold on;
    
    plot(time_seconds, zeros(size(time_seconds)), 'k:', 'DisplayName', 'Zero error', 'HandleVisibility', 'off');
    xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
    
    max_abs_err = max(abs(err_position(:, i)));
    if max_abs_err < 1e-9
        ylim([-0.001, 0.001]);
    else
        buffer = max_abs_err * 0.1;
        ylim([-max_abs_err - buffer, max_abs_err + buffer]);
    end

    hold off;
    xlabel('Time (s)');
    ylabel(['Error ' comp_names{i} ' (m)']);
    title(['Robot position error - component ' comp_names{i}]);
    legend show;
    grid on;
end
sgtitle('Robot end-effector position error over time');

%% --- Plot 4-7: EE orientation error analysis --- 
% --- Plot 4: ORIENTATION error MAGNITUDE (theta) ---
figure('Name', 'Orientation error magnitude');
angle_err_deg_vec = angle_err_rad_vec;
plot(time_seconds, angle_err_deg_vec, 'k', 'LineWidth', 1.5);
hold on; xline(fine_reference_time, 'k--', 'End of robot ref.'); hold off;
title('Orientation error magnitude (Equivalent rotation angle)');
xlabel('Time (s)');
ylabel('Error angle [rad]');
grid on;

% --- Plot 5: ORIENTATION error DIRECTION (u) ---
figure('Name', 'Orientation error direction');
plot(time_seconds, axis_err_mat(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Axis X');
hold on;
plot(time_seconds, axis_err_mat(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Axis Y');
plot(time_seconds, axis_err_mat(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Axis Z');
xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
hold off;
title('Orientation error direction (unit axis components)');
xlabel('Time (s)');
ylabel('Unit axis component');
legend('show', 'Location', 'best'); grid on; ylim([-1.1, 1.1]);


% --- Plot 6: ORIENTATION error VECTOR ---
%              Combines magnitude and direction in a single 3D vector
%              (error metric minimized by the MPC)
figure('Name', 'MPC error vector');
plot(time_seconds, err_orient_vec_mat(:,1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Vector error X');
hold on;
plot(time_seconds, err_orient_vec_mat(:,2), 'g', 'LineWidth', 1.5, 'DisplayName', 'Vector error Y');
plot(time_seconds, err_orient_vec_mat(:,3), 'b', 'LineWidth', 1.5, 'DisplayName', 'Vector error Z');
xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
hold off;
title('Orientation error vector (minimized by MPC)');
xlabel('Time (s)');
ylabel('Vector component value [rad]');
legend('show', 'Location', 'best'); grid on;

% --- Plot 7: Algebraic error of quaternion components (comparison) ---
figure('Name', 'Quaternion algebraic error');
plot(time_seconds, algebraic_quat_error_mat(:,1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Error q_x');
hold on;
plot(time_seconds, algebraic_quat_error_mat(:,2), 'g', 'LineWidth', 1.5, 'DisplayName', 'Error q_y');
plot(time_seconds, algebraic_quat_error_mat(:,3), 'b', 'LineWidth', 1.5, 'DisplayName', 'Error q_z');
plot(time_seconds, algebraic_quat_error_mat(:,4), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'DisplayName', 'Error q_w');
xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
hold off;
title('Algebraic error of quaternion components (q_{ref} - q_{pred, aligned})');
xlabel('Time (s)'); ylabel('Component error');
legend('show', 'Location', 'best'); grid on;

%% --- Plot 8: 3D visualization of orientation error ---
% This plot shows the robot trajectory in 3D space and, for each point,
% draws a vector representing the rotation axis of the error.
% The vector length is proportional to the error angle.

figure('Name', '3D orientation error visualization');
hold on;

% 1. Plot the reference trajectory (black dotted) and the robot's (green)
plot3(pos_ref_data(:,1), pos_ref_data(:,2), pos_ref_data(:,3), 'k:', 'LineWidth', 2, 'DisplayName', 'Reference');
plot3(pos_log_data(:,1), pos_log_data(:,2), pos_log_data(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'EE executed trajectory');

% 2. Use quiver3 to plot the error axis at each step
% Define a scaling factor to make the vectors visible but not too large
axis_scaling_factor = 1; % Tune this value

% Plot a vector every 'plot_every_n_steps' to avoid cluttering the plot
plot_every_n_steps = 2; % E.g. plot a vector every 2 timesteps

for i = 1:plot_every_n_steps:num_timesteps
    % Starting point of the vector (the EE position at that instant)
    start_point = pos_log_data(i, :);
    
    % Vector direction (the unit error axis)
    axis_direction = axis_err_mat(i, :);
    
    % Error magnitude (the angle in radians)
    angle_magnitude = angle_err_rad_vec(i);
    
    % Compute the vector components to plot (direction * magnitude * scaling)
    vector_to_plot = axis_direction * angle_magnitude * axis_scaling_factor;
    
    % Draw the vector using quiver3
    quiver3(start_point(1), start_point(2), start_point(3), ...
            vector_to_plot(1), vector_to_plot(2), vector_to_plot(3), ...
            'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
end

% Plot settings for good visualization
hold off;
title('Orientation error along the trajectory');
xlabel('X axis (m)');
ylabel('Y axis (m)');
zlabel('Z axis (m)');
legend('Reference', 'EE executed trajectory');
grid on;
axis equal;
view(3);     % Set 3D view
rotate3d on; % Enable mouse rotation of the plot

% Create a text string with the explanation
vector_explanation = {'Red vectors:', ...
                      '- Direction: Error rotation axis', ...
                      '- Length: Proportional to error angle [rad]'};
% Add a textbox annotation to the plot
annotation('textbox', [0.15, 0.1, 0.1, 0.1], ... % Position [x, y, width, height] normalized on the figure
           'String', vector_explanation, ...
           'BackgroundColor', [0.95 0.95 0.95], ... % Slightly gray background color
           'FitBoxToText', 'on', ...
           'EdgeColor', 'black');

%% --- Plot 9: Control actions --- 
% Joint velocities (Control commands u_k)
figure('Name', 'Control actions (joint velocities)');
hold on;
plot_styles = {'-', '--', ':', '-.', '-', '--'};
plot_colors = {'b', 'r', 'g', [0.8500 0.3250 0.0980], 'm', [0.4940 0.1840 0.5560]}; % blue, red, green, orange, magenta, purple

for i = 1:6
    plot(time_seconds, u_log_data(:, i), 'LineWidth', 1.5, ...
         'LineStyle', plot_styles{i}, 'Color', plot_colors{i}, ...
         'DisplayName', ['u_' num2str(i)]);
end

xline(fine_reference_time, 'k--', 'End of robot ref.', 'HandleVisibility', 'off');
hold off;
title('Applied control actions');
xlabel('Time (s)');
ylabel('Joint velocity [rad/s]');
legend('show', 'Location', 'best');
grid on;

% NOTE: Joint velocity limits set (rad/s):
% (+-) 10.0, 8.0, 8.0, 2.0, 2.0, 2.0