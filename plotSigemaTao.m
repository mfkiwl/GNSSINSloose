%% 量化噪声
t1=[0:0.1:1];
y1=-t1+10;
%% 角度随机游走
t2=[1:0.1:2];
y2=-t2/2+19/2;
%% 相关噪声


%% 正弦噪声


%% 偏置误差
t4=[3:0.1:4];
y4=5;
%% 速率随机游走 
t5=[4:0.1:5];
y5=t5/2+3;
%% rate ramp
t6=[5:0.1:6];
y6=t6+1/2;

%% PLOT
figure;

plot(t1,y1,'k','LineWidth',2);
hold on;grid on;
plot(t2,y2,'k','LineWidth',2);
hold on;grid on;
plot(t4,y4,'k','LineWidth',2);
hold on;grid on;
plot(t5,y5,'k','LineWidth',2);
hold on;grid on;
plot(t6,y6,'k','LineWidth',2);
hold on;grid on;
