a=load('0321Disu.mat');
% %��ȷ������
Lng=gps.lon;
Lat=gps.lat;
Time=(1:length(Lng)).';

%����
lngfitcoeff = polyfit(Time,Lng,20);
lngfitCurve = polyval(lngfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LngRes=(gps.lon-lngfitCurve).*3600*25*180/pi;
LngMean=mean(LngRes)
LngRMS=rms(LngRes)


%γ��
latfitcoeff = polyfit(Time,Lat,20);
latfitCurve = polyval(latfitcoeff,Time);    %latfitcoeff(5)*latTime.^4+latfitcoeff(4)*latTime.^3+latfitcoeff(3)*latTime.^2+latfitcoeff(2)*latTime+latfitcoeff(1);
LatRes=(gps.lat-latfitCurve)*3600*30*180/pi;
LatMean=mean(LatRes)
LatRMS=rms(LatRes)

figure;
plot(Lng,'-k','linewidth',2);
set(gca, 'Fontname', '��������','FontSize',20);grid on;
hold on;
title('����');
plot(lngfitCurve,'--r','linewidth',2);
%γ��
figure;
plot(Lat,'-k','linewidth',2);
set(gca, 'Fontname', '��������','FontSize',20);grid on;
hold on;
title('γ��');
plot(latfitCurve,'--r','linewidth',2);


figure;
plot(LngRes,'-r','linewidth',2);
set(gca, 'Fontname', '��������','FontSize',20);grid on;
title('��ϵ�����Ϲ켣���Ȳв�');
xlabel('ʱ����Ԫ��s��');
ylabel('��϶�λ���Ȳвm��');

figure;
plot(LatRes,'-r','linewidth',2);
set(gca, 'Fontname', '��������','FontSize',20);grid on;
title('��ϵ�����Ϲ켣γ�Ȳв�');
xlabel('ʱ����Ԫ��s��');
ylabel('��϶�λγ�Ȳвm��');

figure;
plot(lngfitCurve,latfitCurve);
set(gca, 'Fontname', '��������','FontSize',20);grid on;
title('��ϵ�����Ϲ켣');
xlabel('���ȣ�deg��');
ylabel('γ��');