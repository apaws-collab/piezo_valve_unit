% motion gen
mo1 = [];
moTraining.numOfMotions = 1;
moTraining.maxRunTime = 120; % [s]
moTraining.minWaitTime = 0.0; % [s]
moTraining.maxWaitTime = 0.3; % [s]
moTraining.minMoveTime = 0.5; % [s]
moTraining.maxMoveTime = 4; % [s]
moTraining.moveToZero = 3; % [s]
moTraining.scale = 1;
moTraining.x0 = 0;
moTraining.xMin = -4;
moTraining.xMax = 4;
moTraining.dxMin = 0.1;
moTraining.dxMax = 4;

mo1 = motiongen_random_step(mo1, moTraining, dta);

% From workspace block
pDesIn1.time = mo1.traj1.values(:,1);
pDesIn1.signals.values = mo1.traj1.values(:,2);
pDesIn1.signals.dimensions = 1;