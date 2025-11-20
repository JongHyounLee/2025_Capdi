%% ================================
%  IMU 무브별 랜덤 시각화
% ================================
clc; clear; close all;

% --- CSV 파일 선택 (한글 경로 호환 안전버전) ---
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

% --- move 열 확인 ---
if ~ismember('move', T.Properties.VariableNames)
    error('move 열이 없습니다. CSV에 move 컬럼이 있는지 확인하세요.');
end

moves = unique(T.move);
moves(moves == 0) = [];  % 0번 move는 제외 (있을 경우)
fprintf('총 %d개의 move 구간 감지\n', numel(moves));

% --- 랜덤 5개 move 선택 ---
rng('shuffle');
sel_moves = datasample(moves, min(30, numel(moves)), 'Replace', false);
fprintf('랜덤 선택된 move: %s\n', num2str(sel_moves'));

% --- 각 선택된 move별 그래프 출력 ---
for i = 1:numel(sel_moves)
    m = sel_moves(i);
    idx = find(T.move == m);
    if isempty(idx), continue; end

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
        legend('ax','ay','az'); grid on;

        % ----- 자이로 -----
        nexttile;
        plot(t, T.(sprintf('IMU%d_gx',imu))(idx), 'r'); hold on;
        plot(t, T.(sprintf('IMU%d_gy',imu))(idx), 'g');
        plot(t, T.(sprintf('IMU%d_gz',imu))(idx), 'b');
        title(sprintf('IMU%d - Gyro', imu));
        xlabel('Timestep'); ylabel('Gyro');
        legend('gx','gy','gz'); grid on;
    end
end
