clc;
clear all;
close all;
format long g;

load lanxiangResult.mat;

Lng=imu1_e.lon*180/pi;
Lat=imu1_e.lat*180/pi;

% ����ʽ���
Time=(1:length(Lng)).';

%����
lngfitcoeff = polyfit(Time,Lng,15);
lngfitCurve = polyval(lngfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LngRes=(Lng-lngfitCurve)*3600*25;
LngMean=mean(LngRes)
LngRMS=rms(LngRes)

%γ��
latfitcoeff = polyfit(Time,Lat,15);
latfitCurve = polyval(latfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LatRes=(Lat-latfitCurve)*3600*30;
LatMean=mean(LatRes)
LatRMS=rms(LatRes)

%����
figure;
plot(Lng,'-k','linewidth',2);
hold on;
plot(lngfitCurve,'--r','linewidth',2);
%γ��
figure;
plot(Lat,'-k','linewidth',2);
hold on;
plot(latfitCurve,'--r','linewidth',2);


figure;
plot(LngRes,'-k','linewidth',2);
xlabel('ʱ����Ԫ��s��');
ylabel('��϶�λ���Ȳвm��');

figure;
plot(LatRes,'-k','linewidth',2);
xlabel('ʱ����Ԫ��s��');
ylabel('��϶�λγ�Ȳвm��');

