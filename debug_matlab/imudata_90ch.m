%% ================================
%  IMU 무브별 RAW + FEATURE + QUAT + REL 시각화 (move 없어도 동작)
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

% --- move 컬럼 존재 여부 확인 ---
has_move = ismember('move', T.Properties.VariableNames);

if has_move
    % ===== move 컬럼이 있는 경우: 무브별로 처리 =====
    moves = unique(T.move);
    moves(moves == 0) = [];  % 0번 move는 제외(있다면)
    fprintf('총 %d개의 move 구간 감지\n', numel(moves));

    % --- 랜덤 N개 move 선택 ---
    rng('shuffle');
    N_SHOW = min(50, numel(moves));  % 최대 50개
    sel_moves = datasample(moves, N_SHOW, 'Replace', false);
    fprintf('랜덤 선택된 move: %s\n', num2str(sel_moves'));

else
    % ===== move 컬럼이 없는 경우: 전체를 하나의 move로 간주 =====
    fprintf('move 열이 없어 전체 구간을 1개의 move로 처리합니다.\n');
    sel_moves = 1;   % 가짜 move 번호 1개
end

% --- 상대 피처 이름 목록 (있으면 두 번째 Figure로 그림) ---
rel_feat_names = {
    'REL_LH_pitch'
    'REL_LH_roll'
    'REL_RH_pitch'
    'REL_RH_roll'
    'REL_LK_pitch'
    'REL_LK_roll'
    'REL_RK_pitch'
    'REL_RK_roll'
    'REL_THIGH_yaw_diff'
    'REL_ANKLE_yaw_diff'
};

% --- 각 선택된 move 별로 Figure 그리기 ---
for i = 1:numel(sel_moves)
    m = sel_moves(i);

    % ----- 인덱스 선택 -----
    if has_move
        idx = find(T.move == m);
        if isempty(idx), continue; end
    else
        % move 없으면 전체 구간 사용
        idx = (1:height(T)).';
    end

    % ----- Timestep -----
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
    if has_move
        fig_name = sprintf('Move %d', m);
    else
        fig_name = 'Full Data (no move)';
    end

    figure('Name', fig_name, 'NumberTitle', 'off');
    % ✅ 4열 → 5열 (Quaternion 추가)
    tl = tiledlayout(imu_count, 5, 'Padding', 'compact', 'TileSpacing', 'compact');

    if has_move
        title(tl, sprintf('Move %d  |  Label %g  |  Person %s', ...
            m, lbl, pid_str), 'FontWeight','bold');
    else
        title(tl, sprintf('Full Data  |  Label %g  |  Person %s', ...
            lbl, pid_str), 'FontWeight','bold');
    end

    % ----- 각 IMU별 5열 플롯 -----
    for imu = 1:imu_count
        %% ===== 1열: Acc (ax, ay, az) =====
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

        %% ===== 2열: Gyro (gx, gy, gz) =====
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

        %% ===== 3열: Magnitude + Jerk =====
        nexttile;
        if ismember(sprintf('IMU%d_a_mag', imu), T.Properties.VariableNames)
            a_mag   = T.(sprintf('IMU%d_a_mag', imu))(idx);
            g_mag   = T.(sprintf('IMU%d_g_mag', imu))(idx);
            jerk_am = T.(sprintf('IMU%d_jerk_a_mag', imu))(idx);

            plot(t, a_mag, 'Color', [0.93 0.69 0.13], 'LineWidth', 2); hold on;  % |a|
            plot(t, g_mag, 'm', 'LineWidth', 1.2);
            plot(t, jerk_am, 'c--', 'LineWidth', 1.2);
            title(sprintf('IMU%d - |a|, |g|, jerk', imu));
            xlabel('Timestep'); ylabel('Value');
            legend('|a|','|g|','jerk(|a|)', 'Location','best'); grid on;
        else
            % FEATURE 없으면 빈 플롯 + 안내
            plot(t, zeros(size(t)));
            title(sprintf('IMU%d - feature 없음', imu));
            xlabel('Timestep'); ylabel('Value');
            grid on;
        end

        %% ===== 4열: 각도/자세 (pitch, roll, yaw or yaw_int) =====
        nexttile;

        has_pitch = ismember(sprintf('IMU%d_pitch', imu), T.Properties.VariableNames);
        has_roll  = ismember(sprintf('IMU%d_roll',  imu), T.Properties.VariableNames);

        yaw_col_int = sprintf('IMU%d_yaw_int', imu);
        yaw_col     = sprintf('IMU%d_yaw', imu);
        has_yaw_int = ismember(yaw_col_int, T.Properties.VariableNames);
        has_yaw     = ismember(yaw_col,     T.Properties.VariableNames);

        if has_pitch
            pitch_ = T.(sprintf('IMU%d_pitch', imu))(idx);
        else
            pitch_ = zeros(size(t));
        end

        if has_roll
            roll_ = T.(sprintf('IMU%d_roll', imu))(idx);
        else
            roll_ = zeros(size(t));
        end

        if has_yaw_int
            yaw_i = T.(yaw_col_int)(idx);   % yaw_int 있으면 이걸 사용
            yaw_label = 'yaw_{int}';
        elseif has_yaw
            yaw_i = T.(yaw_col)(idx);       % 없으면 yaw 사용
            yaw_label = 'yaw';
        else
            yaw_i = zeros(size(t));
            yaw_label = 'yaw(?)';
        end

        plot(t, pitch_, 'r'); hold on;
        plot(t, roll_,  'b');
        plot(t, yaw_i,  'Color', [0.93 0.69 0.13], 'LineWidth', 1.4);  % 밝은 주황색

        title(sprintf('IMU%d - Pitch/Roll/Yaw', imu));
        xlabel('Timestep'); ylabel('Angle / Int');
        legend('pitch','roll',yaw_label, 'Location','best'); grid on;

        %% ===== 5열: Quaternion (qw, qx, qy, qz) =====
        nexttile;

        has_qw = ismember(sprintf('IMU%d_qw', imu), T.Properties.VariableNames);
        has_qx = ismember(sprintf('IMU%d_qx', imu), T.Properties.VariableNames);
        has_qy = ismember(sprintf('IMU%d_qy', imu), T.Properties.VariableNames);
        has_qz = ismember(sprintf('IMU%d_qz', imu), T.Properties.VariableNames);

        if has_qw && has_qx && has_qy && has_qz
            qw_ = T.(sprintf('IMU%d_qw', imu))(idx);
            qx_ = T.(sprintf('IMU%d_qx', imu))(idx);
            qy_ = T.(sprintf('IMU%d_qy', imu))(idx);
            qz_ = T.(sprintf('IMU%d_qz', imu))(idx);

            plot(t, qw_, 'k'); hold on;
            plot(t, qx_, 'r');
            plot(t, qy_, 'g');
            plot(t, qz_, 'b');
            title(sprintf('IMU%d - Quaternion', imu));
            xlabel('Timestep'); ylabel('q');
            legend('qw','qx','qy','qz', 'Location','best'); grid on;
        else
            plot(t, zeros(size(t)));
            title(sprintf('IMU%d - quat 없음', imu));
            xlabel('Timestep'); ylabel('q');
            grid on;
        end

    end % for imu

    %% ===== REL_* 상대 feature 전용 Figure (있을 때만) =====
    %  (각 move마다 해당 구간만 잘라서 그림)
    rel_exist_mask = ismember(rel_feat_names, T.Properties.VariableNames);
    rel_to_plot = rel_feat_names(rel_exist_mask);

    if ~isempty(rel_to_plot)
        fig_rel_name = [fig_name ' - REL features'];
        figure('Name', fig_rel_name, 'NumberTitle', 'off');

        n_rel = numel(rel_to_plot);
        % 보기 좋게 2열로 배치
        n_cols = 2;
        n_rows = ceil(n_rel / n_cols);

        tl_rel = tiledlayout(n_rows, n_cols, 'Padding', 'compact', 'TileSpacing', 'compact');

        if has_move
            title(tl_rel, sprintf('Move %d - Relative Features', m), 'FontWeight','bold');
        else
            title(tl_rel, 'Full Data - Relative Features', 'FontWeight','bold');
        end

        for j = 1:n_rel
            fname = rel_to_plot{j};
            nexttile;
            y = T.(fname)(idx);
            plot(t, y, 'LineWidth', 1.3);
            title(strrep(fname, '_', '\_'));
            xlabel('Timestep');
            ylabel('Value');
            grid on;
        end
    end

end % for each move
