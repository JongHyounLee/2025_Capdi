%% ================================
%  IMU 무브별 랜덤 시각화 (move/Timestep 없어도 동작)
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
if imu_count == 0
    error('IMU*_ax 형식의 컬럼을 찾지 못했습니다.');
end

% --- move 열 보정 (없으면 전체를 move=1로) ---
if ~ismember('move', T.Properties.VariableNames)
    warning('move 열이 없어 전체를 move=1 하나로 취급합니다.');
    T.move = ones(height(T), 1);
end

% --- Timestep 열 보정 (없으면 1:N 인덱스로) ---
if ~ismember('Timestep', T.Properties.VariableNames)
    warning('Timestep 열이 없어 행 인덱스를 Timestep으로 사용합니다.');
    T.Timestep = (1:height(T))';
end

% --- move 목록 추출 ---
moves = unique(T.move);
moves(moves == 0) = [];  % 0번 move는 있을 경우 제외
fprintf('총 %d개의 move 구간 감지\n', numel(moves));
if isempty(moves)
    error('move 값이 0만 있거나 유효한 move 구간이 없습니다.');
end

% --- 랜덤 최대 10개 move 선택 ---
rng('shuffle');
if numel(moves) <= 10
    sel_moves = moves;
else
    sel_moves = datasample(moves, 10, 'Replace', false);
end
fprintf('랜덤 선택된 move: %s\n', num2str(sel_moves'));

% --- 각 선택된 move별 그래프 출력 ---
for i = 1:numel(sel_moves)
    m = sel_moves(i);
    idx = (T.move == m);
    if ~any(idx)
        continue;
    end

    t = T.Timestep(idx);

    figure('Name', sprintf('Move %d', m), 'NumberTitle', 'off');
    tiledlayout(imu_count, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    for imu = 1:imu_count
        % ----- 가속도 -----
        nexttile;
        plot(t, T.(sprintf('IMU%d_ax',imu))(idx), 'r'); hold on;
        plot(t, T.(sprintf('IMU%d_ay',imu))(idx), 'g');
        plot(t, T.(sprintf('IMU%d_az',imu))(idx), 'b');
        title(sprintf('IMU%d - Acc', imu));
        xlabel('Timestep'); ylabel('Accel');
        legend({'ax','ay','az'}, 'Location','best'); grid on;

        % ----- 자이로 -----
        nexttile;
        plot(t, T.(sprintf('IMU%d_gx',imu))(idx), 'r'); hold on;
        plot(t, T.(sprintf('IMU%d_gy',imu))(idx), 'g');
        plot(t, T.(sprintf('IMU%d_gz',imu))(idx), 'b');
        title(sprintf('IMU%d - Gyro', imu));
        xlabel('Timestep'); ylabel('Gyro');
        legend({'gx','gy','gz'}, 'Location','best'); grid on;
    end
end
