%% IMU UART Logger Script (IMU1/IMU2 한 CSV, 행=센서, 열=ax..gz)
clc; clear; close all;

% (구) serial 객체 청소 (serialport에는 영향 없지만 남겨도 무방)
delete(instrfindall); %#ok<*DLINSTR>

%% === UART 설정 ===
port = "COM3";         % <-- 실제 포트로 변경
baud = 115200;

% 포트 열기 (타임아웃/종단문자 설정)
s = serialport(port, baud, "Timeout", 2);   % 2초 읽기 타임아웃
configureTerminator(s, "CR/LF");            % MCU가 \r\n 이면 CR/LF, \n이면 "LF"
flush(s);

%% === 수집 버퍼 ===
sensorCol = strings(0,1);       % "IMU1" / "IMU2"
valsMat   = zeros(0,6);         % [ax ay az gx gy gz]

%% === UI 생성 ===
fig = uifigure('Name','IMU Logger','Position',[100 100 300 100]);
uibutton(fig,'push','Text','종료',...
    'Position',[100 30 100 40],...
    'ButtonPushedFcn', @(~,~) setappdata(fig,'stopFlag',true));
setappdata(fig,'stopFlag',false);

disp("[INFO] Reading " + port + " @ " + baud + " ...  (종료 버튼으로 멈춤)");

%% === 메인 루프 ===
while true
    % 종료 버튼 확인
    if getappdata(fig,'stopFlag'); break; end

    % 수신 데이터 있으면 한 줄 읽기
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));     % 한 줄(\r\n 기준) 읽기
        if strlength(line)==0, continue; end
        disp(line);                       % 화면에도 보여줌 (원치 않으면 주석)

        % 기대 형식: IMU1,-3064,15249,-25440,-136,378,2
        parts = split(line, ",");
        if numel(parts) ~= 7
            continue;                     % 다른 로그 섞이면 무시
        end

        sensor = upper(strtrim(parts{1}));
        if ~(sensor=="IMU1" || sensor=="IMU2" || sensor=="IMU3")
            continue;
        end

        nums = str2double(parts(2:end));  % [ax ay az gx gy gz]
        if any(isnan(nums)) || numel(nums) ~= 6
            continue;
        end

        % 버퍼에 추가 (행=센서, 열=ax..gz)
        sensorCol(end+1,1) = sensor;            %#ok<AGROW>
        valsMat(end+1, :)  = nums.';            %#ok<AGROW>
    end

    pause(0.01);   % 10ms 틱 (20Hz 수집이면 0.5초는 너무 깁니다)
end

%% === 종료/저장 ===
try, clear s; end

% CSV 파일명
timestamp = datestr(now,'yyyy-mm-dd_HH-MM-SS');
filename = sprintf('imu_rows_%s.csv', timestamp);

% 테이블로 저장 (열: sensor, ax..gz)
T = table(sensorCol, ...
          valsMat(:,1), valsMat(:,2), valsMat(:,3), ...
          valsMat(:,4), valsMat(:,5), valsMat(:,6), ...
          'VariableNames', {'sensor','ax','ay','az','gx','gy','gz'});

writetable(T, filename);
disp(['[INFO] 저장 완료: ' filename]);

try, close(fig); end
