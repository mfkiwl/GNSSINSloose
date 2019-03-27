function [XC]=db5Wavelet(InData)

% InData:输入数据
% level：分解层数
% wavetype：小波类型
% thretype：阈值过滤类型
% SHOR：‘h’/'s'
ThreType='rigrsure';
WaveType='db5';
level=9;
SORH='s';
[C,L]=wavedec(InData,level,WaveType);


cA3=appcoef(C,L,WaveType,9);
cD1=detcoef(C,L,1);
cD2=detcoef(C,L,2);
cD3=detcoef(C,L,3);
cD4=detcoef(C,L,4);
cD5=detcoef(C,L,5);
cD6=detcoef(C,L,6);
cD7=detcoef(C,L,7);
cD8=detcoef(C,L,8);
cD9=detcoef(C,L,9);
switch ThreType
    case 'rigrsure'
        th1_rigrsure=thselect(cD1,'rigrsure');
        th2_rigrsure=thselect(cD2,'rigrsure');
        th3_rigrsure=thselect(cD3,'rigrsure');
        th4_rigrsure=thselect(cD4,'rigrsure');
        th5_rigrsure=thselect(cD5,'rigrsure');
        th6_rigrsure=thselect(cD6,'rigrsure');
        th7_rigrsure=thselect(cD7,'rigrsure');
        th8_rigrsure=thselect(cD8,'rigrsure');
        th9_rigrsure=thselect(cD9,'rigrsure');
        TR_rigrsure=[th1_rigrsure,th2_rigrsure,th3_rigrsure,th4_rigrsure,th5_rigrsure,th6_rigrsure,th7_rigrsure,th8_rigrsure,th9_rigrsure];
        [XC,CXC,LXC,PERF0,PERF2]=wdencmp('lvd',InData,WaveType,9,TR_rigrsure,SORH);
    case 'heursure'
        %-----------------'heursure'------------------------
        th1_heursur=thselect(cD1,'heursure');
        th2_heursur=thselect(cD2,'heursure');
        th3_heursur=thselect(cD3,'heursure');
        th4_heursur=thselect(cD4,'heursure');
        th5_heursur=thselect(cD5,'heursure');
        th6_heursur=thselect(cD6,'heursure');
        th7_heursur=thselect(cD7,'heursure');
        th8_heursur=thselect(cD8,'heursure');
        th9_heursur=thselect(cD9,'heursure');
        TR_heursure=[th1_heursur,th2_heursur,th3_heursur,th4_heursur,th5_heursur,th6_heursur,th7_heursur,th8_heursur,th9_heursur];
         [XC,CXC,LXC,PERF0,PERF2]=wdencmp('lvd',InData,WaveType,9,TR_heursure,SORH);
    case 'sqtwolog'
        %-----------------TPTR = 'sqtwolog', threshold is sqrt(2*log(length(X))).-
        th1_sqtwolog=thselect(cD1,'sqtwolog');
        th2_sqtwolog=thselect(cD2,'sqtwolog');
        th3_sqtwolog=thselect(cD3,'sqtwolog');
        th4_sqtwolog=thselect(cD4,'sqtwolog');
        th5_sqtwolog=thselect(cD5,'sqtwolog');
        TR_sqtwolog=[th1_sqtwolog,th2_sqtwolog,th3_sqtwolog,th4_sqtwolog,th5_sqtwolog];
      [XC,CXC,LXC,PERF0,PERF2]=wdencmp('lvd',InData,WaveType,5,TR_sqtwolog,SORH);
    case 'minimaxi'
        %--------------'minimaxi', minimax thresholding.-------------------
        th1_minimaxi=thselect(cD1,'minimaxi');
        th2_minimaxi=thselect(cD2,'minimaxi');
        th3_minimaxi=thselect(cD3,'minimaxi');
        th4_minimaxi=thselect(cD4,'minimaxi');
        th5_minimaxi=thselect(cD5,'minimaxi');
        TR_minimax=[th1_minimaxi,th2_minimaxi,th3_minimaxi,th4_minimaxi,th5_minimaxi];
       [XC,CXC,LXC,PERF0,PERF2]=wdencmp('lvd',InData,WaveType',5,TR_minimax,SORH);
        %-----------------------------------------------------------------------------
end


