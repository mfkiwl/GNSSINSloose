%GPS/INS无反馈位置组合 卡尔曼滤波器

%%%%%%%%%%%%%%%%%%
%edit by horsejun
%%%%%%%%%%%%%%%%%%

%每秒更新一次速度位置误差
%连续状态系统方程
%dx = F*x + G*w
%z = H*x + v
%离散状态系统方程
%x(k+1) = A*x(k) + B*w(k)
%z(k+1) = C*x(k+1) + v(k+1)

function [E_attitude, E_velocity, E_position, PP] = kalman_GPS_INS_position_sp_NFb(Dp, v, p, quat, Fn, Q, R, Tg, Ta, tao)

%输入
%Dp     量测位置误差， 作为滤波器输入,
%Dv     量测速度误差， 作为滤波器输入，
%p      ins输出位置，作为滤波器系统参数
%v      ins输出速度，作为滤波器系统参数
%fn     ins输出导航系下比力，作为滤波器参数
%quat   ins输出四元数，作为滤波器参数
%Q      系统噪声方差
%R      测量噪声方差
%Ta     加表误差漂移相关时间
%Tg     陀螺仪误差漂移相关时间
%tao    迭代步长
%%%%%%%输入向量均为行向量%%%%%%%%%%%%%

%输出
%E_position     位置预测值
%E-velocity     速度预测值

%各参数初始化
Re 		= 6378245;   %地球长半径
e 		= 1/298.257;  %地球扁率
wie 	= 7.292e-5;  %地球自转角速度

%   东北天速度
Ve0 	= v(:,1);
Vn0   = v(:,2);
Vu0   = v(:,3);
%   导航位置
L0    = p(:,1);
h0    = p(:,3);

%卡尔曼滤波参数初始化
PP(1:18,1:18) = diag([1/(36*57) 1/(36*57) 1/57, 0.0001 0.0001 0.0001, 0 0 1, 0.1/(57*3600) 0.1/(57*3600) 0.1/(57*3600), 0.04/(57*3600) 0.04/(57*3600) 0.04/(57*3600), 1e-4 1e-4 1e-4].^2);   %初始误差协方差阵
PP0 					= PP;
X 						= zeros(18,1);  %初始状态
E_attitude 		= zeros(1,3);
E_position 		= zeros(1,3);
E_velocity 		= zeros(1,3);

n = size(Dp,1);
for i=1:n-1
    %参数赋值
    Ve 		= Ve0(i);
    Vn 		= Vn0(i);
    Vu 		= Vu0(i);
    L 		= L0(i);
    h 		= h0(i);
    fe 		= Fn(i,1);
    fn 		= Fn(i,2);
    fu 		= Fn(i,3);
    Rm 		= Re*(1-2*e+3*e*sin(L)^2);
    Rn 		= Re*(1-e*sin(L)^2);
    %由四元数计算姿态阵
    q 		= quat(i,:);
    Cnb 	= [1-2*(q(3)^2+q(4)^2),     2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
           	 2*(q(2)*q(3)+q(1)*q(4)), 1-2*(q(2)^2+q(4)^2),     2*(q(3)*q(4)-q(1)*q(2));
             2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 1-2*(q(2)^2+q(3)^2)];

    %连续系统状态转换阵 F 的时间更新
    F            = zeros(18,18);
    F(1,2)       = wie*sin(L)+Ve*tan(L)/(Rn+h);
    F(1,3)       = -(wie*cos(L)+Ve/(Rn+h));
    F(1,5)       = -1/(Rm+h);
    F(1,9)       = Vn/(Rm+h)^2;
    F(2,1)       = -(wie*sin(L)+Ve*tan(L)/(Rn+h));
    F(2,3)       = -Vn/(Rm+h);
    F(2,4)       = 1/(Rn+h);
    F(2,7)       = -wie*sin(L);
    F(2,9)       = -Ve/(Rn+h)^2;
    F(3,1)       = wie*cos(L)+Ve/(Rn+h);
    F(3,2)       = Vn/(Rm+h);
    F(3,4)       = tan(L)/(Rn+h);
    F(3,7)       = wie*cos(L)+Ve*(sec(L)^2)/(Rn+h);
    F(3,9)       = -Ve*tan(L)/(Rn+h)^2;
    F(4,2)       = -fu;
    F(4,3)       = fn;
    F(4,4)       = Vn*tan(L)/(Rm+h)-Vu/(Rm+h);
    F(4,5)       = 2*wie*sin(L)+Ve*tan(L)/(Rn+h);
    F(4,6)       = -(2*wie*cos(L)+Ve/(Rn+h));
    F(4,7)       = 2*wie*cos(L)*Vn+Ve*Vn*sec(L)^2/(Rn+h)+2*wie*sin(L)*Vu;
    F(4,9)       = (Ve*Vu-Ve*Vn*tan(L))/(Rn+h)^2;
    F(5,1)       = fu;
    F(5,3)       = -fe;
    F(5,4)       = -2*(wie*sin(L)+Ve*tan(L)/(Rn+h));
    F(5,5)       = -Vu/(Rm+h);
    F(5,6)       = -Vn/(Rm+h);
    F(5,7)       = -(2*wie*cos(L)+Ve*(sec(L)^2)/(Rn+h))*Ve;
    F(5,9)       = (Ve^2*tan(L)+Vn*Vu)/(Rn+h)^2;
    F(6,1)       = -fn;
    F(6,2)       = fe;
    F(6,4)       = 2*(wie*cos(L)+Ve/(Rn+h));
    F(6,5)       = 2*Vn/(Rm+h);
    F(6,7)       = -2*Ve*wie*sin(L);
    F(6,9)       = -(Vn^2+Ve^2)/(Rn+h)^2;
    F(7,5)       = 1/(Rm+h);
    F(7,9)       = -Vn/(Rm+h)^2;
    F(8,4)       = 1/((Rn+h)*cos(L));
    F(8,7)       = Ve*tan(L)/((Rn+h)*cos(L));
    F(8,9)       = -Ve/(cos(L)*(Rn+h)^2);
    F(9,6)       = 1;
    F(1:3,10:12) = Cnb;
    F(1:3,13:15) = Cnb;
    F(4:6,16:18) = Cnb;
    F(13,13)     = -1/Tg(1);
    F(14,14)     = -1/Tg(2);
    F(15,15)     = -1/Tg(3);
    F(16,16)     = -1/Ta(1);
    F(17,17)     = -1/Ta(2);
    F(18,18)     = -1/Ta(3);
    %连续系统输入矩阵更新
    G 				   = zeros(18,9);
    G(1:3,1:3)   = Cnb;
    G(13:15,4:6) = eye(3,3);
    G(16:18,7:9) = eye(3,3);
    %连续系统量测阵更新
    H 					= zeros(3,18);
    H(1,7) 			= 1;
    H(2,8) 			= 1;
    H(3,9) 			= 1;
    %连续系统离散化
    A 				  = eye(18,18)+F*tao;
    B 			    = (eye(18,18)+tao*F/2)*G*tao;
    
    %卡尔曼滤波
    P 					= A*(PP0)*A'+B*Q*B';
    K 					= P*H'*inv(H*P*H'+R);
    PP0 				= (eye(18,18)-K*H)*P;
    PP0 				= (PP0+PP0')/2;
    PP(i,:) 		= diag(PP0);
    
    z = Dp(i+1,:)';
    XX = A*X+K*(z-H*A*X);
    X = XX;

    E_attitude(i+1,:) = XX(1:3)';
    E_velocity(i+1,:) = XX(4:6)';
    E_position(i+1,:) = XX(7:9)';
end

