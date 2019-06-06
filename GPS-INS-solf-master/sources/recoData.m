clc,clear
close all;
%% CODE EXECUTION PARAMETERS

PLOT      = 'ON';   % Plot results.

if (~exist('PLOT','var')),      PLOT = 'OFF'; end

%% CONVERSION CONSTANTS

G = 9.7944;           % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h
%% LOAD REFERENCE DATA

fprintf('开始进行整体数据装载：\n')
%蓝天立交
load 0321lanxiang
allData=LanXiang0321;

% %另一段高速不能用，时间对齐有问题
% load('Gaosu0320.mat')
% allData=Gaosu0320;

% %高速隧道
% load('GaosuSuidao0321.mat');
% allData=GaosuSuidao0321;
% % %低速镇子里
% load 0321Disu;
% allData=x0321DisuData;
%直线走
% load LineWalk190305;
% allData=LineWalk190305;


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
			AngxData(countxAng,:)=ESF(i,1:3)*D2R;
            countxAng=countxAng+1;
		case 13
			AngyData(countyAng,:)=ESF(i,1:3)*D2R;
            countyAng=countyAng+1;
		case 5
			AngzData(countzAng,:)=ESF(i,1:3)*D2R;
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

lengthPVT   =  length(PVT);
PvtData.t   =   (0:1:lengthPVT-1)';
PvtData.lon =   deg2rad(PVT(:,2)/10000000);
PvtData.lat =   deg2rad(PVT(:,3)/10000000);
PvtData.h   =   PVT(:,4)/1000;
PvtData.vel =   [PVT(:,5),PVT(:,6),PVT(:,7)]/1000;
PvtData.std   =   [0.1,0.2,0.1];
PvtData.stdm  =   3.*[2,2,1];
PvtData.stdv  =   0.1 * KT2MS .* [1,1,1]*1.5;
% PvtData.stdv  = [std(PvtData.vel)];
PvtData.larm  =   [1,1,1]';
PvtData.freq  =   1;

% save PvtData;
RefPvt=PvtData;

% % % 
InStart=100;
InEnd=InStart+15;
PvtData.lon(InStart:InEnd)=0;
PvtData.lat(InStart:InEnd)=0;
PvtData.h(InStart:InEnd)=0;
PvtData.vel(InStart:InEnd,:)=0;


gps=PvtData;

OrGps=gps;
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
InsData.freq=10;
InsData.t=(0:0.1:(lengthPVT-3.8))';
% InsData.t=InsData.t(1:3223);

% InsData.arw=[0.008255506,0.013632449,0.010756899];
InsData.arw          = [0.008256,0.013632,0.010757];%2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
InsData.vrw          = 0.3 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
InsData.ab_drift    = 0.5 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
InsData.gb_drift   = 0.008 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
InsData.ab_corr    = 200 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
InsData.gb_corr    = 200 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
InsData.astd=[0.008212535,0.011965155,0.113044401];
InsData.gstd=[0.024667756,0.04169835,0.033167337];
InsData.ab_fix=1.*[0.316398922,0.170514641,0.284741488];%改了个位，0->1
InsData.gb_fix=1.*[0.086656974,0.055781713,0.026144786];
InsData.apsd=[0.01962,0.01962,0.01962];
InsData.gpsd=[0.001221730,0.001221730,0.001221730];
InsData.ini_align=[1 1 1] .* D2R;
InsData.ini_align_err=[1,1,1]*5;

%数据试验区
% InsData.arw          = 2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
% InsData.vrw          = 0.3 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
% InsData.ab_drift    = 0.3 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
% InsData.gb_drift   = 0.008 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
% InsData.ab_corr    = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
% InsData.gb_corr    = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
% InsData.astd=0.005.*ones(1,3);%[0.008212535,0.011965155,0.013044401];
% InsData.gstd=0.005.*ones(1,3);%[0.024667756,0.04169835,0.033167337];
% InsData.ab_fix=0.05.*ones(1,3);%[0.316398922,0.170514641,1.284741488];
% InsData.gb_fix=0.05.*ones(1,3);%[0.086656974,0.055781713,0.026144786];
% InsData.apsd=0.01.*ones(1,3);%[0.01962,0.01962,0.01962];
% InsData.gpsd=0.01.*ones(1,3);%[0.001221730,0.001221730,0.001221730];
% InsData.ini_align=[10,10,10];
% InsData.ini_align_err=[10,10,10];
dt = mean(diff(InsData.t));                    % IMU mean period
imu1 = imu_err_profile(InsData, dt);      % Transform IMU manufacturer error units to SI units.

% imu1.ini_align_err = [1 1 10] .* D2R;                     % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  

% save InsData;



%% MU8 wavelet denoise
% function [XC,CXC,LXC,PERF0,PERF2]=db5Wavelet(InData,level,WaveType,ThreType,SORH)
% db5Wavelet(InData,level,WaveType,ThreType,SORH)

% InData:输入数据
% level：分解层数
% wavetype：小波类型
% thretype：阈值过滤类型
% SHOR：‘h’/'s'
fprintf('开始进行小波滤波处理：\n')

OutFb1=db5Wavelet(imu1.fb(:,1),'bior3.3','s','rigrsure');
OutFb2=db5Wavelet(imu1.fb(:,2),'bior3.3','s','rigrsure');
OutFb3=db5Wavelet(imu1.fb(:,3),'bior3.3','s','rigrsure');
OutWb1=db5Wavelet(imu1.wb(:,1),'db5','s','rigrsure');
OutWb2=db5Wavelet(imu1.wb(:,2),'db5','s','rigrsure');
OutWb3=db5Wavelet(imu1.wb(:,3),'db5','s','rigrsure');
% 
imu1.fb=[OutFb1,OutFb2,OutFb3];
imu1.fb=imu1.fb(1:2759,:);
imu1.wb=[OutWb1,OutWb2,OutWb3];

% gps.lon=db5Wavelet(gps.lon,'bior3.3');
% gps.lat=db5Wavelet(gps.lat,'bior3.3');
% gps.h=db5Wavelet(gps.h,'bior3.3');
% gps.vel(:,1)=db5Wavelet(gps.vel(:,1),'bior3.3');
% gps.vel(:,2)=db5Wavelet(gps.vel(:,2),'bior3.3');
% gps.vel(:,3)=db5Wavelet(gps.vel(:,3),'bior3.3');
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
   [imu1_e] = ins_gps(imu1, gps, 'quaternion','double');
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


%% IMU1_E GRAVITY

imu1_e.gravity=gravity(imu1_e.lat,imu1_e.h);

%% PLOT

fprintf('开始绘制结果图：\n')

%smoothdata(a,'gaussian',10);
if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.P_d.^(0.5)).*3;
   
    % VELOCITIES
    figure;
    subplot(311)
    plot( OrGps.t, OrGps.vel(:,1),'-y','LineWidth',4);hold on;
    plot( imu1_e.t, smooth(imu1_e.vel(:,1)),':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('NORTH VELOCITY');
    
    subplot(312)
    plot( OrGps.t, OrGps.vel(:,2),'-y','LineWidth',4);hold on;
    plot(imu1_e.t, imu1_e.vel(:,2),':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('EAST VELOCITY');
    
    subplot(313)
    plot(OrGps.t, OrGps.vel(:,3),'-y','LineWidth',4);hold on;
    plot(imu1_e.t, imu1_e.vel(:,3),':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('DOWN VELOCITY');

    % POSITION
    figure;
    subplot(311)
    plot( OrGps.t, OrGps.lat.*R2D, '-y','LineWidth',4);hold on;
    plot( imu1_e.t, imu1_e.lat.*R2D, ':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[deg]')
    legend( 'PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('纬度');
    
    subplot(312)
    plot( OrGps.t,OrGps.lon.*R2D, '-y','LineWidth',4);hold on;
    plot(imu1_e.t, imu1_e.lon.*R2D, ':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('经度');
    
    subplot(313)
    plot( OrGps.t,OrGps.h, '-y', 'LineWidth',4);hold on;
    plot(imu1_e.t, imu1_e.h, ':r','LineWidth',4);
    xlabel('Time [s]')
    ylabel('[m]')
    legend('PVT', 'PVT/MEAS');
    set(gca, 'Fontname', '华文中宋','FontSize',14);
    title('高度');
    
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
   plot3(OrGps.lon.*R2D, OrGps.lat.*R2D,OrGps.h,'-y','LineWidth',4);
   xlabel('经度');
   ylabel('纬度');
   zlabel('高度');
   title('3维轨迹图');
   set(gca, 'Fontname', '华文中宋','FontSize',14);
   grid on;
   hold on;
   plot3(imu1_e.lon.*R2D, imu1_e.lat.*R2D,imu1_e.h,':r','LineWidth',4);
   legend('PVT','PVT/MEAS');
   
   figure;
   plot(OrGps.lon.*R2D, OrGps.lat.*R2D,'-y','LineWidth',4);
   hold on;grid on;
   plot(imu1_e.lon.*R2D, imu1_e.lat.*R2D,':r','LineWidth',4);
   set(gca, 'Fontname', '华文中宋','FontSize',14);
   legend('PVT','PVT/MEAS');
   title('2维轨迹对比图');
   
   figure;
   plot(imu1_e.lon.*R2D, imu1_e.lat.*R2D);
   set(gca, 'Fontname', '华文中宋','FontSize',14);grid on;
   title('组合导航平面轨迹结果');
end


%% 画地图
GnssRel=[PvtData.lon,PvtData.lat];
save('GnssRel.mat', 'GnssRel');
InsRel=[imu1_e.lon*R2D,imu1_e.lat*R2D];
save ('InsRel.mat', 'InsRel');
aaa=load('GnssRel.mat')
save_to_js({'InsRel.mat'},{'red'},[2],[1]);

%% 数据误差分析STD
slNum=80;
R2D=1;
STD.gpsLon=std(OrGps.lon);
STD.gpsLat=std(OrGps.lat);
STD.gpsh=std(OrGps.h);
STD.gpsVel=[std(OrGps.vel(:,1)),std(OrGps.vel(:,2)),std(OrGps.vel(:,2))];
STD.IGlon=std(imu1_e.lon(slNum:length(imu1_e.lon)));
STD.IGlat=std(imu1_e.lat(slNum:length(imu1_e.lat)));
STD.IGh=std(imu1_e.h(slNum:length(imu1_e.h)));
STD.IGVel=[std(imu1_e.vel(slNum:length(imu1_e.vel),1)),std(imu1_e.vel(slNum:length(imu1_e.vel),2)),std(imu1_e.vel(slNum:length(imu1_e.vel),3))];
% Print STD &RMS
fprintf('开始打印GNSS/INS STD：\n');
Exl_STD_GPS_LLH=[STD.gpsLon,STD.gpsLat,STD.gpsh];
Exl_STD_GPS_Vel=[STD.gpsVel(1),STD.gpsVel(2),STD.gpsVel(3)];
Exl_STD_IG_LLH=[STD.IGlon,STD.IGlat,STD.IGh];
Exl_STD_IG_Vel=[STD.IGVel(1),STD.IGVel(2),STD.IGVel(3)];
Exl_STD_LLH=[Exl_STD_GPS_LLH;Exl_STD_IG_LLH];
Exl_STD_Vel=[Exl_STD_GPS_Vel;Exl_STD_IG_Vel];

xlswrite('GNSS_INS_STD.xlsx',Exl_STD_LLH,'Sheet1','B2');  
xlswrite('GNSS_INS_STD.xlsx',Exl_STD_Vel,'Sheet2','B2'); 
%% 数据误差分析RMS
RMS.gpsLon=rms(OrGps.lon-mean(OrGps.lon));
RMS.gpsLat=rms(OrGps.lat-mean(OrGps.lat));
RMS.gpsh=rms(OrGps.h-mean(gps.h));
RMS.gpsVel=[rms(OrGps.vel(:,1)-mean(OrGps.vel(:,1))),rms(OrGps.vel(:,2)-mean(OrGps.vel(:,2))),rms(OrGps.vel(:,2)-mean(OrGps.vel(:,2)))];
RMS.IGlon=rms(imu1_e.lon(slNum:length(imu1_e.lon))-mean(imu1_e.lon(slNum:length(imu1_e.lon))));
RMS.IGlat=rms(imu1_e.lat(slNum:length(imu1_e.lat))-mean(imu1_e.lat(slNum:length(imu1_e.lat))));
RMS.IGh=rms(imu1_e.h(slNum:length(imu1_e.h))-mean(imu1_e.h(slNum:length(imu1_e.h))));
RMS.IGVel=[rms(imu1_e.vel(slNum:length(imu1_e.vel),1)-mean(imu1_e.vel(slNum:length(imu1_e.vel),1))),rms(imu1_e.vel(slNum:length(imu1_e.vel),2)-mean(imu1_e.vel(slNum:length(imu1_e.vel),2))),rms(imu1_e.vel(slNum:length(imu1_e.vel),3)-mean(imu1_e.vel(slNum:length(imu1_e.vel),3)))];
% fprintf('STD:',STD);

%% print IMU Error to excel chart
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
% ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
fprintf('开始打印IMU误差：\n');
ExlImu_arw=[InsData.arw(1),imu1.arw(1),InsData.arw(2),imu1.arw(2),InsData.arw(3),imu1.arw(3)];
ExlImu_vrw=[InsData.vrw(1),imu1.vrw(1),InsData.vrw(2),imu1.vrw(2),InsData.vrw(3),imu1.vrw(3)];
ExlImu_gstd=[InsData.gstd(1),imu1.gstd(1),InsData.gstd(2),imu1.gstd(2),InsData.gstd(3),imu1.gstd(3)];
ExlImu_astd=[InsData.astd(1),imu1.astd(1),InsData.astd(2),imu1.astd(2),InsData.astd(3),imu1.astd(3)];
ExlImu_gFix=[InsData.gb_fix(1),imu1.gb_fix(1),InsData.gb_fix(2),imu1.gb_fix(2),InsData.gb_fix(3),imu1.gb_fix(3)];
ExlImu_aFix=[InsData.ab_fix(1),imu1.ab_fix(1),InsData.ab_fix(2),imu1.ab_fix(2),InsData.ab_fix(3),imu1.ab_fix(3)];
ExlImu_gDrift=[InsData.gb_drift(1),imu1.gb_drift(1),InsData.gb_drift(2),imu1.gb_drift(2),InsData.gb_drift(3),imu1.gb_drift(3)];
ExlImu_aDrift=[InsData.ab_drift(1),imu1.ab_drift(1),InsData.ab_drift(2),imu1.ab_drift(2),InsData.ab_drift(3),imu1.ab_drift(3)];
ExlImu_gCorr=[InsData.gb_corr(1),imu1.gb_corr(1),InsData.gb_corr(2),imu1.gb_corr(2),InsData.gb_corr(3),imu1.gb_corr(3)];
ExlImu_aCorr=[InsData.ab_corr(1),imu1.ab_corr(1),InsData.ab_corr(2),imu1.ab_corr(2),InsData.ab_corr(3),imu1.ab_corr(3)];
ExlImu_gPSD=[InsData.gpsd(1),imu1.gpsd(1),InsData.gpsd(2),imu1.gpsd(2),InsData.gpsd(3),imu1.gpsd(3)];
ExlImu_aPSD=[InsData.apsd(1),imu1.apsd(1),InsData.apsd(2),imu1.apsd(2),InsData.apsd(3),imu1.apsd(3)];
ExlImu_ini_align=[InsData.ini_align(1),imu1.ini_align(1),InsData.ini_align(2),imu1.ini_align(2),InsData.ini_align(3),imu1.ini_align(3)];
ExlImu_ini_align_err=[InsData.ini_align_err(1),imu1.ini_align_err(1),InsData.ini_align_err(2),imu1.ini_align_err(2),InsData.ini_align_err(3),imu1.ini_align_err(3)];
EXL_Imu=[ExlImu_arw;ExlImu_vrw;ExlImu_gstd;ExlImu_astd;ExlImu_gFix;ExlImu_aFix;ExlImu_gDrift;ExlImu_aDrift;ExlImu_gCorr;ExlImu_aCorr;ExlImu_gPSD;ExlImu_aPSD;ExlImu_ini_align;ExlImu_ini_align_err];
% xlswrite('imuErr.xlsx',EXL_Imu,'Sheet1','B3');  
fprintf('IMU误差已经写好。\n');

Org=imu1_e;

%% 结果比较

fprintf('与PVT结果比较');
Tt=0:0.1:length(RefPvt.lon)-14.1;
TT=0:1:length(RefPvt.lon);
%卫星定位结果比较
Lng=RefPvt.lon;
Lat=RefPvt.lat;
Time=(1:length(Lng)).';

LngRes=(Org.lon(1:10:length(Org.lon))-Lng(1:276)).*3600*25*R2D;
%未丢失情况下的RMS计算
LngMean=mean(LngRes)
LngRMS=rms(LngRes)


%纬度
LatRes=(Org.lat(1:10:length(Org.lon))-Lat(1:276))*3600*30*R2D;
%未丢失情况下的RMS计算源
LatMean=mean(LatRes)
LatRMS=rms(LatRes)
TwoDRMS=sqrt(LatRMS^2+LngRMS^2)

fprintf('与PVT结果比较局部RMS');
LatRMSPart=rms(LatRes(InStart:InEnd+20))
LngRMSPart=rms(LngRes(InStart:InEnd+20))

TwoDRMS=sqrt(LatRMSPart^2+LngRMSPart^2)


figure;
plot(LngRes,'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航与PVT经度残差');
xlabel('时间历元（s）');
ylabel('组合定位经度残差（m）');

figure;
plot(LatRes,'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航与PVT纬度残差');
xlabel('时间历元（s）');
ylabel('组合定位纬度残差（m）');


fprintf('与拟合结果比较');
%与组合导航结果拟合结果比较
%组合导航结果田王立交
a=load('RefInsPvt.mat')
% % 高速另一段
% a=load('Gaosu0320.mat')

%高速隧道
% a=load('GaosuSuidao0321.mat')

% 低速镇子里
% a=load('0321Disu.mat');
% %正确解算结果
Lng=imu1_e.lon;
Lat=imu1_e.lat;
Time=(1:length(Lng)).';

%经度
lngfitcoeff = polyfit(Time,Lng,15);
lngfitCurve = polyval(lngfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LngRes=(imu1_e.lon-lngfitCurve).*3600*25*180/pi;
LngMean=mean(LngRes)
LngRMS=rms(LngRes)
LngSTD=std(LngRes)

%纬度
latfitcoeff = polyfit(Time,Lat,15);
latfitCurve = polyval(latfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LatRes=(imu1_e.lat-latfitCurve)*3600*30*180/pi;
LatMean=mean(LatRes)
LatRMS=rms(LatRes)
LatSTD=std(LatRes)
TwoDRMS=sqrt(LatRMS^2+LngRMS^2)

fprintf('拟合结果局部RMS');
LatRMSPart=rms(LatRes(InStart:InEnd+20))
LngRMSPart=rms(LngRes(InStart:InEnd+20))

TwoDRMS=sqrt(LatRMSPart^2+LngRMSPart^2)

figure;
plot(Tt,LngRes(1:2700),'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹经度残差');
xlabel('时间历元（s）');
ylabel('组合定位经度残差（m）');

figure;
plot(Tt,LatRes(1:2700),'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹纬度残差');
xlabel('时间历元（s）');
ylabel('组合定位纬度残差（m）');

figure;
plot(lngfitCurve*180/pi,latfitCurve*180/pi);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹');
xlabel('经度（deg）');
ylabel('纬度');