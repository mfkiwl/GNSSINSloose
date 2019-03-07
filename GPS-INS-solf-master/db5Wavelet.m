function [WaveOut]=db5Wavelat(InData,level,WaveType)

[C,L]=wavedec(InData,level,WaveType);

%%提取小波系数
%从信号中提取小波系数，cD1为各层小波系数
cA3=appcoef(C,L,'db5',5);
cD1=detcoef(C,L,1);
cD2=detcoef(C,L,2);
cD3=detcoef(C,L,3);
cD4=detcoef(C,L,4);
cD5=detcoef(C,L,5);

%%隔层阈值选择
%使用stein的五偏似然估计原理进行各层阈值选择
%-----------------‘rigrsure'----------------------为无偏似然估计阈值类型，SORH为选择阈值类型
th1_rigrsure=thselect(cD1,'rigrsure');
th2_rigrsure=thselect(cD2,'rigrsure');
th3_rigrsure=thselect(cD3,'rigrsure');
th4_rigrsure=thselect(cD4,'rigrsure');
th5_rigrsure=thselect(cD5,'rigrsure');
TR_rigrsure=[th1_rigrsure,th2_rigrsure,th3_rigrsure,th4_rigrsure,th5_rigrsure];
%-----------------'heursure'------------------------
th1_heursur=thselect(cD1,'heursure');
th2_heursur=thselect(cD2,'heursure');
th3_heursur=thselect(cD3,'heursure');
th4_heursur=thselect(cD4,'heursure');
th5_heursur=thselect(cD5,'heursure');
TR_heursure=[th1_heursur,th2_heursur,th3_heursur,th4_heursur,th5_heursur];
%-----------------TPTR = 'sqtwolog', threshold is sqrt(2*log(length(X))).-
th1_sqtwolog=thselect(cD1,'sqtwolog');
th2_sqtwolog=thselect(cD2,'sqtwolog');
th3_sqtwolog=thselect(cD3,'sqtwolog');
th4_sqtwolog=thselect(cD4,'sqtwolog');
th5_sqtwolog=thselect(cD5,'sqtwolog');
TR_sqtwolog=[th1_sqtwolog,th2_sqtwolog,th3_sqtwolog,th4_sqtwolog,th5_sqtwolog];
%--------------'minimaxi', minimax thresholding.-------------------
th1_minimaxi=thselect(cD1,'minimaxi');
th2_minimaxi=thselect(cD2,'minimaxi');
th3_minimaxi=thselect(cD3,'minimaxi');
th4_minimaxi=thselect(cD4,'minimaxi');
th5_minimaxi=thselect(cD5,'minimaxi');
TR_minimax=[th1_minimaxi,th2_minimaxi,th3_minimaxi,th4_minimaxi,th5_minimaxi];
%-----------------------------------------------------------------------------
SORH='h';%软阈值

%---------denoise----------

%XC为去噪后信号
%[CXC,LXC]为小波的分解结构
%PERF0& PERF2是恢复和压缩的范数百分比 
%’lvd'为允许设置的各层阈值
%'gbl'为固定阈值
%3为阈值的长度

%%小波重构
[XC_rigrsure,CXC_rigrsure,LXC_rigrsure,PERF0_rigrsure,PERF2_rigrsure]=wdencmp('lvd',InData,'db5',5,TR_rigrsure,SORH);
[XC_heursure,CXC_heursure,LXC_heursure,PERF0_heursure,PERF2_heursure]=wdencmp('lvd',InData,'db5',5,TR_heursure,SORH);
[XC_sqtwolog,CXC_sqtwolog,LXC_sqtwolog,PERF0_sqtwolog,PERF2_sqtwolog]=wdencmp('lvd',InData,'db5',5,TR_sqtwolog,SORH);
[XC_minimax,CXC_minimax,LX_minimax,PERF0_minimax,PERF2_minimax]=wdencmp('lvd',InData,'db5',5,TR_minimax,SORH);
%-----------denoise measure  SNR bigger is better MSE smaller is better