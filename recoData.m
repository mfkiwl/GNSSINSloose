clc,clear
load LineWalk190305;
allData=LineWalk190305;
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
InsData.wb=[AngxData(:,3),AngyData(:,3),AngzData(:,3)];
InsData.fb=[AccxData(:,3),AccyData(:,3),AcczData(:,3)];
%InsData.t=AngzData(,:1);
InsData.freq=10;

lengthPVT=length(PVT);
PvtData.t=PVT(:,1);
PvtData.lon=PVT(:,2)/10000000;
PvtData.lat=PVT(:,3)/10000000;
PvtData.h=PVT(:,4)/1000;
PvtData.vel=[PVT(:,5),PVT(:,6),PVT(:,7)]/1000;
InsData.t=(PvtData.t(1)/1000:0.1:PvtData.t(lengthPVT)/1000)';
InsData.t=InsData.t(1:3223);


