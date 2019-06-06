clc;
clear all;
close all;
format long g;

load lanxiangResult.mat;

Lng=imu1_e.lon*180/pi;
Lat=imu1_e.lat*180/pi;

% 多项式拟合
Time=(1:length(Lng)).';

%经度
lngfitcoeff = polyfit(Time,Lng,15);
lngfitCurve = polyval(lngfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LngRes=(Lng-lngfitCurve)*3600*25;
LngMean=mean(LngRes)
LngRMS=rms(LngRes)

%纬度
latfitcoeff = polyfit(Time,Lat,15);
latfitCurve = polyval(latfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LatRes=(Lat-latfitCurve)*3600*30;
LatMean=mean(LatRes)
LatRMS=rms(LatRes)

%经度
figure;
plot(Lng,'-k','linewidth',2);
hold on;
plot(lngfitCurve,'--r','linewidth',2);
%纬度
figure;
plot(Lat,'-k','linewidth',2);
hold on;
plot(latfitCurve,'--r','linewidth',2);


figure;
plot(LngRes,'-k','linewidth',2);
xlabel('时间历元（s）');
ylabel('组合定位经度残差（m）');

figure;
plot(LatRes,'-k','linewidth',2);
xlabel('时间历元（s）');
ylabel('组合定位纬度残差（m）');

