%% ================================
%  IMU 무브별 RAW + FEATURE 시각화
% ================================
clc; clear; close all;

% --- CSV 파일 선택 ---
[filename, pathname] = uigetfile('*.csv', 'IMU 데이터 파일 선택');
if isequal(filename,0)
    error('파일을 선택하지 않았습니다.');
end
csv_path = fullfile(pathname, filename);
T = readtable(csv_path);

% --- IMU 개수 자동 감지 ---
imu_count = 0;
for k = 1:10
    if ismember(sprintf('IMU%d_ax',k), T.Properties.VariableNames)
        imu_count = imu_count + 1;
    else
        break;
    end
end
fprintf('감지된 IMU: %d개\n', imu_count);

% --- move 컬럼 확인 ---
if ~ismember('move', T.Properties.VariableNames)
    error('move 열이 없습니다. CSV에 move 컬럼이 있는지 확인하세요.');
end

moves = unique(T.move);
moves(moves == 0) = [];  % 0번 move는 제외(있다면)
fprintf('총 %d개의 move 구간 감지\n', numel(moves));

% --- 랜덤 N개 move 선택 ---
rng('shuffle');
N_SHOW = min(5, numel(moves));  % 최대 10개
sel_moves = datasample(moves, N_SHOW, 'Replace', false);
fprintf('랜덤 선택된 move: %s\n', num2str(sel_moves'));

% --- 각 선택된 move 별로 Figure 그리기 ---
for i = 1:numel(sel_moves)
    m   = sel_moves(i);
    idx = find(T.move == m);
    if isempty(idx), continue; end

    % Timestep
    if ismember('Timestep', T.Properties.VariableNames)
        t = T.Timestep(idx);
    else
        t = (0:numel(idx)-1).';  % 없는 경우 그냥 0,1,2,...
    end

    % ----- label -----
    if ismember('label', T.Properties.VariableNames)
        lbl_vals = T.label(idx);
        if isnumeric(lbl_vals) || islogical(lbl_vals)
            lbl = mode(lbl_vals);
        else
            lbl = NaN;
        end
    else
        lbl = NaN;
    end

    % ----- person_id -----
    if ismember('person_id', T.Properties.VariableNames)
        pid_vals = T.person_id(idx);

        if isnumeric(pid_vals) || islogical(pid_vals)
            pid_str = sprintf('%g', mode(pid_vals));
        else
            % string / cellstr / categorical 등 처리
            u = unique(pid_vals);
            pid_str = string(u(1));   % 첫 번째 값만 사용
        end
    else
        pid_str = "N/A";
    end

    % ----- Figure / Layout 생성 -----
    figure('Name', sprintf('Move %d', m), 'NumberTitle', 'off');
    tl = tiledlayout(imu_count, 4, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf('Move %d  |  Label %g  |  Person %s', ...
        m, lbl, pid_str), 'FontWeight','bold');

    % ----- 각 IMU별 4열 플롯 -----
    for imu = 1:imu_count
        % ===== 1열: Acc (ax, ay, az) =====
        nexttile;
        ax_ = T.(sprintf('IMU%d_ax', imu))(idx);
        ay_ = T.(sprintf('IMU%d_ay', imu))(idx);
        az_ = T.(sprintf('IMU%d_az', imu))(idx);
        plot(t, ax_, 'r'); hold on;
        plot(t, ay_, 'g');
        plot(t, az_, 'b');
        title(sprintf('IMU%d - Acc', imu));
        xlabel('Timestep'); ylabel('Accel (LSB)');
        legend('ax','ay','az', 'Location','best'); grid on;

        % ===== 2열: Gyro (gx, gy, gz) =====
        nexttile;
        gx_ = T.(sprintf('IMU%d_gx', imu))(idx);
        gy_ = T.(sprintf('IMU%d_gy', imu))(idx);
        gz_ = T.(sprintf('IMU%d_gz', imu))(idx);
        plot(t, gx_, 'r'); hold on;
        plot(t, gy_, 'g');
        plot(t, gz_, 'b');
        title(sprintf('IMU%d - Gyro', imu));
        xlabel('Timestep'); ylabel('Gyro (LSB)');
        legend('gx','gy','gz', 'Location','best'); grid on;

        % ===== 3열: Magnitude + Jerk =====
        nexttile;
        a_mag   = T.(sprintf('IMU%d_a_mag', imu))(idx);
        g_mag   = T.(sprintf('IMU%d_g_mag', imu))(idx);
        jerk_am = T.(sprintf('IMU%d_jerk_a_mag', imu))(idx);

        plot(t, a_mag, 'k', 'LineWidth', 1); hold on;
        plot(t, g_mag, 'm', 'LineWidth', 1);
        plot(t, jerk_am, 'c--');   % Jerk는 점선
        title(sprintf('IMU%d - |a|, |g|, jerk', imu));
        xlabel('Timestep'); ylabel('Value');
        legend('|a|','|g|','jerk(|a|)', 'Location','best'); grid on;

        % ===== 4열: 각도/자세 (pitch, roll, yaw_int) =====
        nexttile;
        pitch_ = T.(sprintf('IMU%d_pitch', imu))(idx);
        roll_  = T.(sprintf('IMU%d_roll',  imu))(idx);
        yaw_i  = T.(sprintf('IMU%d_yaw_int', imu))(idx);

        plot(t, pitch_, 'r'); hold on;
        plot(t, roll_,  'b');
plot(t, yaw_i,  'Color', [0.93 0.69 0.13], 'LineWidth', 1.4);  % ★ 밝은 주황색
        title(sprintf('IMU%d - Pitch/Roll/YawInt', imu));
        xlabel('Timestep'); ylabel('Angle / Int');
        legend('pitch','roll','yaw_{int}', 'Location','best'); grid on;
    end
end
