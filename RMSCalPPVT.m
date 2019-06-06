a=load('0321Disu.mat');
% %正确解算结果
Lng=gps.lon;
Lat=gps.lat;
Time=(1:length(Lng)).';

%经度
lngfitcoeff = polyfit(Time,Lng,20);
lngfitCurve = polyval(lngfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LngRes=(gps.lon-lngfitCurve).*3600*25*180/pi;
LngMean=mean(LngRes)
LngRMS=rms(LngRes)


%纬度
latfitcoeff = polyfit(Time,Lat,20);
latfitCurve = polyval(latfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LatRes=(gps.lat-latfitCurve)*3600*30*180/pi;
LatMean=mean(LatRes)
LatRMS=rms(LatRes)

figure;
plot(Lng,'-k','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
hold on;
title('经度');
plot(lngfitCurve,'--r','linewidth',2);
%纬度
figure;
plot(Lat,'-k','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
hold on;
title('纬度');
plot(latfitCurve,'--r','linewidth',2);


figure;
plot(LngRes,'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹经度残差');
xlabel('时间历元（s）');
ylabel('组合定位经度残差（m）');

figure;
plot(LatRes,'-r','linewidth',2);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹纬度残差');
xlabel('时间历元（s）');
ylabel('组合定位纬度残差（m）');

figure;
plot(lngfitCurve,latfitCurve);
set(gca, 'Fontname', '华文中宋','FontSize',20);grid on;
title('组合导航拟合轨迹');
xlabel('经度（deg）');
ylabel('纬度');