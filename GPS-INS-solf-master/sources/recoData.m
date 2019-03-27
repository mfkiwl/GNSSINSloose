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

fprintf('��ʼ������������װ�أ�\n')

load 0321LanXiang;
allData=LanXiang0321;
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

fprintf('��ʼ����PVT���ݽ����\n')

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
PvtData.std   =   [std(PvtData.lon),std(PvtData.lat),std(PvtData.h)];
PvtData.stdm  =   1.*[1,1,1];
PvtData.stdv  =   0.1 * KT2MS .* [0.5,0.5,1];%[std(PvtData.vel)];
PvtData.larm  =   [0,0,0]';
PvtData.freq  =   1;

% save PvtData;
% % 
InStart=50;
InEnd=InStart+15;
PvtData.lon(InStart:InEnd)=0;
PvtData.lat(InStart:InEnd)=0;
PvtData.h(InStart:InEnd)=0;
PvtData.vel(InStart:InEnd,:)=0;

gps=PvtData;


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
InsData.arw          = 5   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
InsData.vrw          = 0.3 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
InsData.ab_drift    = 0.5 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
InsData.gb_drift   = 0.008 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
InsData.ab_corr    = 200 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
InsData.gb_corr    = 200 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
InsData.astd=[0.008212535,0.011965155,0.113044401];
InsData.gstd=[0.024667756,0.04169835,0.033167337];
InsData.ab_fix=3.*[0.316398922,0.170514641,0.284741488];%���˸�λ��0->1
InsData.gb_fix=3.*[0.086656974,0.055781713,0.026144786];
InsData.apsd=[0.01962,0.01962,0.01962];
InsData.gpsd=[0.001221730,0.001221730,0.001221730];
InsData.ini_align=[1 1 1] .* D2R;
InsData.ini_align_err=[1,1,1]*5;

%����������
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

imu1.ini_align_err = [1 1 5] .* D2R;                     % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  

% save InsData;



%% MU8 wavelet denoise
% function [XC,CXC,LXC,PERF0,PERF2]=db5Wavelet(InData,level,WaveType,ThreType,SORH)
% db5Wavelet(InData,level,WaveType,ThreType,SORH)

% InData:��������
% level���ֽ����
% wavetype��С������
% thretype����ֵ��������
% SHOR����h��/'s'
fprintf('��ʼ����С���˲�����\n')

OutFb1=db5Wavelet(imu1.fb(:,1));
OutFb2=db5Wavelet(imu1.fb(:,2));
OutFb3=db5Wavelet(imu1.fb(:,3));
OutWb1=db5Wavelet(imu1.wb(:,1));
OutWb2=db5Wavelet(imu1.wb(:,2));
OutWb3=db5Wavelet(imu1.wb(:,3));

imu1.fb=[OutFb1,OutFb2,OutFb3];
imu1.fb=imu1.fb(1:2759,:);
imu1.wb=[OutWb1,OutWb2,OutWb3];


%% INS/GPS integration using 
fprintf('��ʼ������ϵ�������\n')
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


%% IMU1_E GRAVITY

imu1_e.gravity=gravity(imu1_e.lat,imu1_e.h);

%% PLOT

fprintf('��ʼ���ƽ��ͼ��\n')

%smoothdata(a,'gaussian',10);
if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.P_d.^(0.5)).*3;
   
    % VELOCITIES
    figure;
    subplot(311)
    plot( gps.t, gps.vel(:,1),':r', imu1_e.t, smooth(imu1_e.vel(:,1)),'-k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MMEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('NORTH VELOCITY');
    
    subplot(312)
    plot( gps.t, gps.vel(:,2),':r', imu1_e.t, imu1_e.vel(:,2),'-k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('EAST VELOCITY');
    
    subplot(313)
    plot(gps.t, gps.vel(:,3),':r', imu1_e.t, imu1_e.vel(:,3),'-k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('PVT', 'MEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('DOWN VELOCITY');

    % POSITION
    figure;
    subplot(311)
    plot( gps.t, gps.lat.*R2D, ':r', imu1_e.t, imu1_e.lat.*R2D, '-k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[deg]')
    legend( 'PVT', 'MEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('γ��');
    
    subplot(312)
    plot( gps.t, gps.lon.*R2D, ':r', imu1_e.t, imu1_e.lon.*R2D, '-k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('PVT', 'MEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('����');
    
    subplot(313)
    plot( gps.t, gps.h, ':r', imu1_e.t, imu1_e.h, 'k','LineWidth',2);
    xlabel('Time [s]')
    ylabel('[m]')
    legend('PVT', 'MEANS');
    set(gca, 'Fontname', '��������','FontSize',14);
    title('�߶�');
    
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
   plot3(gps.lon.*R2D, gps.lat.*R2D,gps.h);
   xlabel('����');
   ylabel('γ��');
   zlabel('�߶�');
   title('3ά�켣ͼ');
   set(gca, 'Fontname', '��������','FontSize',14);
   grid on;hold on;
   hold on;
   plot3(imu1_e.lon.*R2D, imu1_e.lat.*R2D,imu1_e.h);
   legend('PVT','PVT/INS');
   
   figure;
   plot(gps.lon.*R2D, gps.lat.*R2D);
   hold on;grid on;
   plot(imu1_e.lon.*R2D, imu1_e.lat.*R2D);
   set(gca, 'Fontname', '��������','FontSize',14);
   legend('PVT','PVT/INS');
   title('2ά�켣�Ա�ͼ');
   
   figure;
   plot(imu1_e.lon.*R2D, imu1_e.lat.*R2D);
   set(gca, 'Fontname', '��������','FontSize',14);grid on;
   title('��ϵ���ƽ��켣���');
end


%% ����ͼ
GnssRel=[PvtData.lon,PvtData.lat];
save('GnssRel.mat', 'GnssRel');
InsRel=[imu1_e.lon*R2D,imu1_e.lat*R2D];
save ('InsRel.mat', 'InsRel');
aaa=load('GnssRel.mat')
save_to_js({'InsRel.mat'},{'red'},[2],[1]);

%% ����������STD
slNum=80;
R2D=1;
STD.gpsLon=std(gps.lon*R2D-mean(gps.lon*R2D));
STD.gpsLat=std(gps.lat*R2D-mean(gps.lat*R2D));
STD.gpsh=std(gps.h-mean(gps.h));
STD.gpsVel=[std(gps.vel(:,1)-mean(gps.vel(:,1))),std(gps.vel(:,2)-mean(gps.vel(:,2))),std(gps.vel(:,2)-mean(gps.vel(:,2)))];
STD.IGlon=std(imu1_e.lon(slNum:length(imu1_e.lon))*R2D-mean(imu1_e.lon(slNum:length(imu1_e.lon))*R2D));
STD.IGlat=std(imu1_e.lat(slNum:length(imu1_e.lat))*R2D-mean(imu1_e.lat(slNum:length(imu1_e.lat))*R2D));
STD.IGh=std(imu1_e.h(slNum:length(imu1_e.h))-mean(imu1_e.h(slNum:length(imu1_e.h))));
STD.IGVel=[std(imu1_e.vel(slNum:length(imu1_e.vel),1)-mean(imu1_e.vel(slNum:length(imu1_e.vel),1))),std(imu1_e.vel(slNum:length(imu1_e.vel),2)-mean(imu1_e.vel(slNum:length(imu1_e.vel),2))),std(imu1_e.vel(slNum:length(imu1_e.vel),3)-mean(imu1_e.vel(slNum:length(imu1_e.vel),3)))];
% Print STD &RMS
fprintf('��ʼ��ӡGNSS/INS STD��\n');
Exl_STD_GPS_LLH=[STD.gpsLon,STD.gpsLat,STD.gpsh];
Exl_STD_GPS_Vel=[STD.gpsVel(1),STD.gpsVel(2),STD.gpsVel(3)];
Exl_STD_IG_LLH=[STD.IGlon,STD.IGlat,STD.IGh];
Exl_STD_IG_Vel=[STD.IGVel(1),STD.IGVel(2),STD.IGVel(3)];
Exl_STD_LLH=[Exl_STD_GPS_LLH;Exl_STD_IG_LLH];
Exl_STD_Vel=[Exl_STD_GPS_Vel;Exl_STD_IG_Vel];

xlswrite('GNSS_INS_STD.xlsx',Exl_STD_LLH,'Sheet1','B2');  
xlswrite('GNSS_INS_STD.xlsx',Exl_STD_Vel,'Sheet2','B2'); 
%% ����������RMS
RMS.gpsLon=rms(gps.lon*R2D-mean(gps.lon*R2D));
RMS.gpsLat=rms(gps.lat*R2D-mean(gps.lat*R2D));
RMS.gpsh=rms(gps.h-mean(gps.h));
RMS.gpsVel=[rms(gps.vel(:,1)-mean(gps.vel(:,1))),rms(gps.vel(:,2)-mean(gps.vel(:,2))),rms(gps.vel(:,2)-mean(gps.vel(:,2)))];
RMS.IGlon=rms(imu1_e.lon(slNum:length(imu1_e.lon))*R2D-mean(imu1_e.lon(slNum:length(imu1_e.lon))*R2D));
RMS.IGlat=rms(imu1_e.lat(slNum:length(imu1_e.lat))*R2D-mean(imu1_e.lat(slNum:length(imu1_e.lat))*R2D));
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
fprintf('��ʼ��ӡIMU��\n');
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
xlswrite('imuErr.xlsx',EXL_Imu,'Sheet1','B3');  
fprintf('IMU����Ѿ�д�á�\n');
 
 

 
 
 