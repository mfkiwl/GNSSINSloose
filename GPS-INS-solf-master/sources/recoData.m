clc,clear

%% CODE EXECUTION PARAMETERS

PLOT      = 'ON';   % Plot results.

if (~exist('PLOT','var')),      PLOT = 'OFF'; end

%% CONVERSION CONSTANTS

G = 9.81;           % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h
%% LOAD REFERENCE DATA

fprintf('开始进行整体数据装载：\n')

load LineWalk190305;
allData=LineWalk190305;
lengthData=length(allData);
cE=1;
cP=1;
for i= 1:lengthData
    if allData(i,1)==2
        ESF(cE,1:3)=allData(i,2:4);
        cE=cE+1;
    end
    if allData(i,1)==1
        PVT(cP,:)=allData(i,2:8);
        cP=cP+1;
    end
    
end

lengthEsf=length(ESF);
countxAng=1;
countyAng=1;
countzAng=1;
countxAcc=1;
countyAcc=1;
countzAcc=1;
for i=1:lengthEsf
	switch ESF(i,2)
		case 14
			AngxData(countxAng,:)=ESF(i,1:3);
            countxAng=countxAng+1;
		case 13
			AngyData(countyAng,:)=ESF(i,1:3);
            countyAng=countyAng+1;
		case 5
			AngzData(countzAng,:)=ESF(i,1:3);
            countzAng=countzAng+1;
		case 16
			AccxData(countxAcc,:)=ESF(i,1:3);
            countxAcc=countxAcc+1;
		case 17
			AccyData(countyAcc,:)=ESF(i,1:3);
            countyAcc=countyAcc+1;
		case 18
			AcczData(countzAcc,:)=ESF(i,1:3);
            countzAcc=countzAcc+1;
	end
end


%% M8U PVT error profile

fprintf('开始进行PVT数据解读：\n')

% PVT data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm (x-right, y-fwd, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).

lengthPVT=length(PVT);
PvtData.t=(0:1:lengthPVT-1)';
PvtData.lon=deg2rad(PVT(:,2)/10000000);
PvtData.lat=deg2rad(PVT(:,3)/10000000);
PvtData.h=PVT(:,4)/1000;
PvtData.vel=[PVT(:,5),PVT(:,6),PVT(:,7)]/1000;
PvtData.std=[std(PvtData.lon),std(PvtData.lat),std(PvtData.h)];
PvtData.stdm=[10,10,10];
PvtData.stdv=[std(PvtData.vel)];
PvtData.larm=[0,0,0]';
PvtData.freq=10;

gps=PvtData;
save PvtData;



%% M8U IMU error profile

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%       vrw: 1x3 angle velocity walks (m/s^2/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%     gpsd : 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%     apsd : 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).

% ref dataset will be used to simulate IMU sensors.

InsData.wb=[deg2rad(AngxData(:,3)),deg2rad(AngyData(:,3)),deg2rad(AngzData(:,3))];
% InsData.wb=[AngxData(:,3),AngyData(:,3),AngzData(:,3)];
InsData.fb=[AccxData(:,3),AccyData(:,3),AcczData(:,3)];
%InsData.t=AngzData(,:1);
InsData.freq=100;
InsData.t=(0:0.1:(lengthPVT-3.8))';
% InsData.t=InsData.t(1:3223);

% InsData.arw=[0.008255506,0.013632449,0.010756899];
InsData.arw=deg2rad([0.008255506,0.013632449,0.010756899]);
InsData.vrw=[0.001434401,0.003960394,0.004316029];
InsData.ab_drift=[0.000536859,0.001549454,0.001255873];
InsData.gb_drift=[0.003469329,0.003907706,0.003030818];
InsData.ab_corr=[10,10,10];
InsData.gb_corr=[10,10,10];
InsData.astd=[0.008212535,0.011965155,0.013044401];
InsData.gstd=[0.024667756,0.04169835,0.033167337];
InsData.ab_fix=[0.316398922,0.170514641,1.284741488];
InsData.gb_fix=[0.086656974,0.055781713,0.026144786];
InsData.apsd=[0.01962,0.01962,0.01962];
InsData.gpsd=[0.001221730,0.001221730,0.001221730];
InsData.ini_align=[0,0,0];
InsData.ini_align_err=[0,0,0];

dt = mean(diff(InsData.t));                    % IMU mean period
imu1 = imu_err_profile(InsData, dt);      % Transform IMU manufacturer error units to SI units.

imu1.ini_align_err = [1 1 5] .* D2R;                     % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  

save InsData;




%% INS/GPS integration using 
fprintf('开始进行组合导航解算\n')
 % Sincronize GPS data with IMU data.
    
    % Guarantee that gps.t(1) < imu1.t(1) < gps.t(2)
    if (imu1.t(1) < gps.t(1)),
        
        igx  = find(imu1.t > gps.t(1), 1, 'first' );
        
        imu1.t  = imu1.t  (igx:end, :);
        imu1.fb = imu1.fb (igx:end, :);
        imu1.wb = imu1.wb (igx:end, :);        
    end
    
    % Guarantee that imu1.t(end-1) < gps.t(end) < imu1.t(end)
    if (imu1.t(end) <= gps.t(end)),
        
        fgx  = find(gps.t < imu1.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
   [imu1_e] = ins_gps(imu1, PvtData, 'quaternion','double');
    % ---------------------------------------------------------------------
save imu1_e.mat imu1_e
% OUTPUT:
%   ins_gps_e, INS/GPS estimates data structure.
%         t: Ix1 time vector (seconds).
%      roll: Ix1 roll (radians).
%     pitch: Ix1 pitch (radians).
%       yaw: Ix1 yaw (radians).
%       vel: Ix3 NED velocities (m/s).
%       lat: Ix1 latitude (radians).
%       lon: Ix1 longitude (radians).
%         h: Ix1 altitude (m).
%       P_d: Mx21 P matrix diagonals.
%         B: Mx12 Kalman filter biases compensations.
%       Inn: Mx6  Kalman filter innovations.
%         X: Mx21 Kalman filter states evolution.

%% PLOT

fprintf('开始绘制结果图：\n')


if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.P_d.^(0.5)).*3;
   
    % VELOCITIES
    figure;
    subplot(311)
    plot( gps.t, gps.vel(:,1),'-c', imu1_e.t, imu1_e.vel(:,1),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MMEANS');
    title('NORTH VELOCITY');
    
    subplot(312)
    plot( gps.t, gps.vel(:,2),'-c', imu1_e.t, imu1_e.vel(:,2),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MEANS');
    title('EAST VELOCITY');
    
    subplot(313)
    plot(gps.t, gps.vel(:,3),'-c', imu1_e.t, imu1_e.vel(:,3),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MEANS');
    title('DOWN VELOCITY');
    
    
    % POSITION
    figure;
    subplot(311)
    plot( gps.t, gps.lat.*R2D, '-c', imu1_e.t, imu1_e.lat.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend( 'PVT', 'MEANS');
    title('纬度');
    
    subplot(312)
    plot( gps.t, gps.lon.*R2D, '-c', imu1_e.t, imu1_e.lon.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('PVT', 'MEANS');
    title('经度');
    
    subplot(313)
    plot( gps.t, gps.h, '-c', imu1_e.t, imu1_e.h, '-b');
    xlabel('Time [s]')
    ylabel('[m]')
    legend('PVT', 'MEANS');
    title('纬度');
    
    % POSITION ERRORS
    % fh = @radicurv;
    % [RNs,REs] = arrayfun(fh, lat_rs);
    
    [RN,RE]  = radius(imu1_e.lat, 'double');
    LAT2M = RN + imu1_e.h;
    LON2M = (RE + imu1_e.h).*cos(imu1_e.lat);
    
    [RN,RE]  = radius(gps.lat, 'double');
    lat2m_g = RN + gps.h;
    lon2m_g = (RE + gps.h).*cos(gps.lat);
    
    figure;
    subplot(311)
    plot (gps.t, gps.lat-imu1_e.lat, '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    title('PVT与组合导航纬度差');
    
    subplot(312)
%     plot(gps.t, lon2m_g.*sig3_rr(:,8), '--k', gps.t, -lon2m_g.*sig3_rr(:,8), '--k' )
    plot (gps.t, gps.lon-imu1_e.lon, '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    title('PVT与组合导航经度差');
    
    subplot(313)
   plot (gps.t, gps.h-imu1_e.h, '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    title('PVT与组合导航高度差');
    
end


