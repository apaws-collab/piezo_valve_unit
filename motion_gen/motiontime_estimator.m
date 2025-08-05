function accTime = motiontime_estimator(motion)
% accumulates the motion time in min
accSec = 0;

nT = fieldnames(motion);
numOfTraj = numel(nT);
for iTraj=1:numOfTraj
    accSec = accSec + motion.(nT{iTraj}).values(end,1);
end
accTime = seconds(accSec);
accTime.Format = 'mm:ss'; 
end