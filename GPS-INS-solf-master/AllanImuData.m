% clc
% clear
% close all
% matlabrc
% 
% load Means022402
%%%%%%%%%%%%%%%%%%%%%%    变量赋�?    %%%%%%%%%%%%%%%%%%%%%%%
esfData=Means030502
lenESF=length(esfData)/6;
EsfxAng=zeros(floor(lenESF),2);
EsfyAng=zeros(floor(lenESF),2);
EsfzAng=zeros(floor(lenESF),2);
EsfxAcc=zeros(floor(lenESF),2);
EsfyAcc=zeros(floor(lenESF),2);
EsfzAcc=zeros(floor(lenESF),2);
CouxAng = 1;
CouyAng = 1;
CouzAng = 1;
CouxAcc = 1;
CouyAcc = 1;
CouzAcc = 1;

%取GYRO\ACC�?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:lenESF
    if esfData(i,2) == 14
        EsfxAng(CouxAng,1)=esfData(i,1)/1000;
    	EsfxAng(CouxAng,2)=esfData(i,3);
        CouxAng=CouxAng+1;
    elseif esfData(i,2) == 13
    	EsfyAng(CouyAng,1)=esfData(i,1)/1000;
    	EsfyAng(CouyAng,2)=esfData(i,3);
        CouyAng=CouyAng+1;
    elseif esfData(i,2) == 5
    	EsfzAng(CouzAng,1)=esfData(i,1)/1000;
    	EsfzAng(CouzAng,2)=esfData(i,3);
        CouzAng=CouzAng+1;
    elseif esfData(i,2) == 16
        EsfxAcc(CouxAcc,1)=esfData(i,1)/1000;
    	EsfxAcc(CouxAcc,2)=esfData(i,3);
        CouxAcc=CouxAcc+1;
    elseif esfData(i,2) == 17
    	EsfyAcc(CouyAcc,1)=esfData(i,1)/1000;
    	EsfyAcc(CouyAcc,2)=esfData(i,3);
        CouyAcc=CouyAcc+1;
    elseif esfData(i,2) == 18
    	EsfzAcc(CouzAcc,1)=esfData(i,1)/1000;
    	EsfzAcc(CouzAcc,2)=esfData(i,3);
        CouzAcc=CouzAcc+1;
    end
end

ImuData.fb=[EsfxAcc(1:(CouxAcc-1),2),EsfyAcc(1:(CouyAcc-1),2),EsfzAcc(1:(CouzAcc-1),2)];
% ImuData.fb=[EsfxAcc(1:(CouxAcc-1),2),EsfyAcc(1:(CouyAcc),2),EsfzAcc(1:(CouzAcc),2)];
% ImuData.fb=[EsfxAcc(1:(CouxAcc),2),EsfyAcc(1:(CouyAcc),2),EsfzAcc(1:(CouzAcc),2)];
ImuData.wb=[EsfxAng(1:(CouxAng-1),2),EsfyAng(1:CouyAng,2),EsfzAng(1:CouzAng,2)];
ImuData.freq=10;
ImuData.t=0:0.1:(CouxAng-2)/10;

[stim300] = allan_imu (ImuData);