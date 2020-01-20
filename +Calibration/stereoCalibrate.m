function stereoParams = stereoCalibrate(modelPtsCell, camImgPtsCell, prjImgPtsCell, camImgSize, prjImgSize, objective, useDistortion)
%% Calibrates camera and projector parameters using bundle adjustment
%
% This function calibrates camera and projector intrinsics and extrinsics
% using bundle adjustment (BA). During the bundle adjustment, we also
% modify the 3d coordinates of the grid points in model space. However,
% performing Least Squares Nonlinear optimization on all intrinsics,
% extrinsics and model points is both unstable and computational expensive
% So we must provide the Jacobian matrix pattern to the lsqnonlin option.
%
% See also: Calibration.calibrateStereoInitGuess

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%% Flags
if nargin < 7
    useDistortion = true;
end

%% 1. Calibrate camera and projector intrinsics and R, T from model points
% Maximum count for termination criteria
maxCount = 1000;

% OpenCV camera/stereo calibrate criterial
calibCriteria = struct('type','Count+EPS', 'maxCount', maxCount, 'epsilon', 1e-6);

% camera init guess
if (~useDistortion) % no tangent or radial distortions, i.e., (p1,p2,k1,k2) = 0
    [camK, camKc, camReprojErr, camRvecs, camTvecs] = cv.calibrateCamera(modelPtsCell, camImgPtsCell, camImgSize, ...
        'Criteria', calibCriteria, 'FixTangentDist', true, 'FixK1', true,'FixK2', true, 'FixK3', true);
    
    % projector init guess
    [prjK, prjKc, prjReprojErr, prjRvecs, prjTvecs] = cv.calibrateCamera(modelPtsCell, prjImgPtsCell, prjImgSize, ...
        'Criteria', calibCriteria, 'FixTangentDist', true, 'FixK1', true,'FixK2', true, 'FixK3', true);
else
    [camK, camKc, camReprojErr, camRvecs, camTvecs] = cv.calibrateCamera(modelPtsCell, camImgPtsCell, camImgSize, ...
        'Criteria', calibCriteria, 'FixK3', true);
    
    % projector init guess
    [prjK, prjKc, prjReprojErr, prjRvecs, prjTvecs] = cv.calibrateCamera(modelPtsCell, prjImgPtsCell, prjImgSize, ...
        'Criteria', calibCriteria, 'FixK3', true);
end

% only keep k1,k2,p1,p2
camKc = camKc(1:4);
prjKc = prjKc(1:4);

%% 2. Setup nonlinear optimization
% 2.1 optimization function and algorithm
options = optimoptions('lsqnonlin');
options.Algorithm = 'trust-region-reflective'; % can use jacobina pattern
% options.Algorithm = 'levenberg-marquardt';
options.FiniteDifferenceType = 'central';

% 2.2 termination criterias
options.MaxIterations = 50;
options.MaxFunctionEvaluations = 1000000;
options.StepTolerance = 1e-7;

% 2.3 results display and plot
options.Diagnostics = 'off';

if(0) % debug
    options.Display = 'iter';
    options.PlotFcn = {@optimplotx,@optimplotfval, @optimplotfirstorderopt, @optimplotresnorm, @optimplotstepsize};
else
    options.Display = 'off';
    options.PlotFcn = [];
end

% 2.4 whether use parallel computing
options.UseParallel = false;

%% 3. Set initial guess of all R, T between cam and model plane
Rs = zeros(3,3, length(camRvecs));
rs = zeros(3,1, length(camRvecs));
Ts = zeros(3,1, length(camRvecs));

RTs = [];
for i=1:length(camRvecs)
    camR = cv.Rodrigues(camRvecs{i});
    prjR = cv.Rodrigues(prjRvecs{i});
    
    Rs(:,:,i) = prjR*camR';
    rs(:,:,i) = cv.Rodrigues(Rs(:,:,i));
    Ts(:,:,i) = prjTvecs{i} - Rs(:,:,i)*camTvecs{i};
    
    % all R, T between cam and prj and cam model
    RTs = [RTs, camRvecs{i}', camTvecs{i}'];
end

% 3.1 take the median as bundle adjustment initial value
r0 = median(rs, 3);
T0 = median(Ts, 3);

% 3.2 set initial paramters to be optimized
param0 = Calibration.unpackStereoParams(camK, camKc, prjK, prjKc);
param0 = [param0, r0', T0', RTs];

if(strcmp(objective, 'withoutBA'))
    % choose objective function
    objFun = @(x)withoutBA(x, modelPtsCell, camImgPtsCell, prjImgPtsCell);
    options.TypicalX = param0;
    
elseif(strcmp(objective, 'BA'))
    % choose objective function
    objFun = @(x)bundleAdjust(x, modelPtsCell, camImgPtsCell, prjImgPtsCell);
    
    % append extra parameters: model points are also optimized
    modelPtsMat = cell2mat(modelPtsCell')';
    param0 = [param0, modelPtsMat(:)'];
    
    % set sparse jacobian pattern
    options.JacobPattern = jacobianPattern(param0, modelPtsCell, useDistortion);
end

%% 4. nonlinear optimization
param = lsqnonlin(objFun,param0,[],[],options);

%% 5. extract optimized paramters
[camK, camKc, prjK, prjKc, R, T] = Calibration.packStereoParams(param);

% 5.1 output cam, prj intrinsics and extrinsics
stereoParams.camK = camK;
stereoParams.camKc = camKc;
stereoParams.prjK = prjK;
stereoParams.prjKc = prjKc;
stereoParams.R = R;
stereoParams.T = T;

% The computed fundamental matrix below transforms a point in camera imgae
% to an epipolar line in projector image. Rotate F if you want to transform
% from projector to camera...

% translation vector in matrix cross product form
tx = [0, -T(3), T(2);
    T(3), 0 , -T(1);
    -T(2), T(1) , 0 ];

% E = [t] × R
stereoParams.E = tx * R;

% F = inv(K2')*E*inv(K1);
stereoParams.F = inv(prjK')*stereoParams.E*inv(camK);

% 5.2 output cam-model extrinsics and optimized grid points world and model
% coords
numSets = length(modelPtsCell);

% grid points in model and world space
stereoParams.modelPts = cell(1, numSets);
stereoParams.worldPts = cell(1, numSets);

% model to cam or prj ri,ti
stereoParams.camRvecs = cell(1, numSets);
stereoParams.camTvecs = cell(1, numSets);
stereoParams.prjRvecs = cell(1, numSets);
stereoParams.prjTvecs = cell(1, numSets);

if(strcmp(objective, 'BA'))
    % 3d points column offset of each plane
    pts3dPos = [0,cumsum(cellfun(@(x) length(x), modelPtsCell))]+1;
    
    % get bundle adjusted grid points in model space
    modelPts = reshape(param(22+6*length(modelPtsCell)+1:end), 3, [])';
end

% for each plane
for i=1:numSets
    if(strcmp(objective, 'withoutBA'))
        curModelPts = modelPtsCell{i};
    elseif(strcmp(objective, 'BA'))
        curModelPts = modelPts(pts3dPos(i):pts3dPos(i+1)-1,:);
    end
    
    stereoParams.modelPts{i} = curModelPts;
    
    % get ri,ti from model to cam
    curExPos = 22+6*(i-1);
    camRvec = param(curExPos+1 : curExPos+3);
    camTvec = param(curExPos+4 : curExPos+6);
    
    % use ri,ti to convert to world space
    stereoParams.worldPts{i} = Calibration.rotateTranslatePoints(curModelPts, camTvec, camRvec, true);
    stereoParams.camRvecs{i} = camRvec;
    stereoParams.camTvecs{i} = camTvec';
    
    % projector extrinsics
    stereoParams.prjRvecs{i} = cv.Rodrigues(stereoParams.R*cv.Rodrigues(camRvec));
    stereoParams.prjTvecs{i} = (stereoParams.R*camTvec' + stereoParams.T);
end

stereoParams.camImgPts = camImgPtsCell;
stereoParams.prjImgPts = prjImgPtsCell;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Objective functions

% without BA model points
function res = withoutBA(paramIn, modelPts, camPts2d, prjPts2d)
% pack parameters from x
[camK, camKc, prjK, prjKc, R, T] = Calibration.packStereoParams(paramIn);

numSets = length(modelPts);

% compute reprojection residual
res = [];
for i=1:numSets
    
    % camera residual
    pos = 22+(i-1)*6 + 1;
    camRvec = paramIn(pos:pos+2);
    camTvec = paramIn(pos+3:pos+5);
    camRes = camPts2d{i} - cv.projectPoints(modelPts{i}, camRvec, camTvec, camK, 'DistCoeffs', camKc);
    
    % projector residual
    Rcam = cv.Rodrigues(camRvec);
    prjRvec = cv.Rodrigues(R*Rcam);
    prjTvec = (R*camTvec'+ T)';
    prjRes = prjPts2d{i} - cv.projectPoints(modelPts{i}, prjRvec, prjTvec, prjK, 'DistCoeffs', prjKc);
    
    % concatenate residual
    res = [res; camRes(:); prjRes(:)];
end

end

%% BA model points
function res = bundleAdjust(paramIn, initModelPts, camPts2dCell, prjPts2dCell)
% pack parameters from x
[camK, camKc, prjK, prjKc, R, T] = Calibration.packStereoParams(paramIn);

% # of planes (poses) during calibration
numSets = length(initModelPts);

% convert vecotr of [X,Y,Z]... to Nx3 mat
pos = 22+6*numSets+1;
modelPts = reshape(paramIn(pos:end), 3, [])';

% compute reprojection residual
res = [];
prevPos = 1;
% gdPtsCamView = [];
% gdPtsPrjView = [];
% lambdaZ = [];
for i=1:numSets
    % current plane's pts3d
    nPts = length(initModelPts{i});
    curModelPts = modelPts(prevPos:prevPos+nPts-1,:);
    prevPos = prevPos+nPts;
    
    % camera residual
    pos = 23+(i-1)*6;
    camRvec = paramIn(pos:pos+2);
    camTvec = paramIn(pos+3:pos+5);
    camRes = camPts2dCell{i} - cv.projectPoints(curModelPts, camRvec, camTvec, camK, 'DistCoeffs', camKc);
    
    % projector residual
    Rcam = cv.Rodrigues(camRvec);
    prjRvec = cv.Rodrigues(R*Rcam);
    prjTvec = (R*camTvec'+ T)';
    prjRes = prjPts2dCell{i} - cv.projectPoints(curModelPts, prjRvec, prjTvec, prjK, 'DistCoeffs', prjKc);
    
    % concatenate cam and prj reprojection residuals
    res = [res; camRes(:); prjRes(:)];
    
end

% obj points scale contraint, avoid model points scale drift, since it
% couples with ri, ti
% make sure the residual is arranged as [X;Y;Z;X;Y;Z...] order
scaleRes = modelPts - cell2mat(initModelPts'); %  Xm_hat - Xm_dot

lambdaZ = exp(-sum(scaleRes.^2, 2));
scaleRes(:,1) = scaleRes(:,1).*lambdaZ;
scaleRes(:,2) = scaleRes(:,2).*lambdaZ;
scaleRes(:,3) = scaleRes(:,3).*lambdaZ;

% concatenate scale constraint to reprojection residual
scaleRes = scaleRes';
res = [res; scaleRes(:)];

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Construct Jacobian pattern for faster bundle adjustment
function J = jacobianPattern(x, modelPts, useDistortion)

%% Descriptions
% This function generates a sparse Jacobian pattern for lsqnonlin, if any
% entry is 0, it means its finite difference is 0, otherwise it is
% computed using finite difference. 

%% NOTE: jacobian pattern is as follows:
%--------------------------------------------------------------------------
%               |[fx_c,fy_c,cx_c,cy_c,k1_c,k2_c,p1_c,p2_c,|fx_p,fy_p,cx_p,cy_p,k1_p,k2_p,p1_p,p2_p,|r01,r02,r03,t01,t02,t03,|ri1,ri2,ri3,ti1,ti2,ti3,|X1,Y1,Z1,....Xn,Yn,Zn]|
% camReprjResx1 |                                         |                                        |                        |                        |                      |
% ....          |                                         |                                        |                        |                        |                      |
% prjReprjResxN |                                         |                                        |                        |                        |                      |
% prjReprjResy1 |                                         |                                        |                        |                        |                      |
% ....          |                                         |                                        |                        |                        |                      |
% prjReprjResyN |                                         |                                        |                        |                        |                      |
% 3dPtsResX1    |                                         |                                        |                        |                        |                      |
% 3dPtsResY1    |                                         |                                        |                        |                        |                      |
% 3dPtsResZ1    |                                         |                                        |                        |                        |                      |
% ....          |                                         |                                        |                        |                        |                      |
% 3dPtsResXN    |                                         |                                        |                        |                        |                      |
% 3dPtsResYN    |                                         |                                        |                        |                        |                      |
% 3dPtsResZN    |                                         |                                        |                        |                        |                      |
%--------------------------------------------------------------------------
% where r0 t0 are R T from cam to prj
% ri, ti (i>0) are R T from model to cam
% We put 1 in the entry only if the derevative of param to residual != 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if we estimate camera and projector intrinsics and extrinsics, and adjust
% 3d points, then we have 8 intrinsics for either cam and prj, and 6
% extrinsics, plus 6xnPlanes R|T, and 3xN params = 22 + 6*nPlanes + 3xN.
cols = size(x, 2);

% # intrinsics
nCams = 2;
nProjElmts = 4; % fx,fy,cx,cy
nDistFactors = 4; % k1,k2,p1,p2
nIntrinsics = nCams * (nProjElmts+nDistFactors);

% # of extrinsics
nExtrinsics = 6; % r0 and t0 between cam and prj

% # of checkerboard poses
numSets = length(modelPts);

% 3d points column offset of each plane
pts3dPos = 3*[0,cumsum(cellfun(@(x) length(x), modelPts))]+1;

% extra extrinsics between cam and calibration planes
nExtrinsicsCam = 6*numSets;

% total intrinsics and extrinsics
nInExParams = nIntrinsics + nExtrinsics + nExtrinsicsCam;

% # of points
nPts = (cols - nInExParams)/3;

% for sum of cam and prj reprojection error, each point provide 4
% equations, camResx, camResy, prjResx, prjResy. And if we constraint 3d
% points to be similar to original points, then another 3xN equations.
rows = 2*nPts + 2*nPts + 3*nPts;
% rows = 2*nPts + 2*nPts;

% set J as all zeros sparse matrix
nz = (14*2+19*2)*nPts; % nonzero element numbers
J = spalloc(rows,cols,nz);

%% set ones entries
% the residual pattern in lsqnonlin is res = [camResx(:), camResy(:),
% prjResx(:), prjResy(:), pts3dResX(:), pts3dResY(:), pts3dResZ(:)];

% for cam points, its partial derivative to prj intrinsics are 0, and vice
% versa
% resxyNonrelated = zeros(1,8);

% d(reprjResxy) / d(r1,r2,r3,t1,t2,t3)
% resxExtrinsic = [1,1,1,1,0,1];
% resyExtrinsic = [1,1,1,0,1,1];

% 2d reprojection error to 3d points, d(reprjResxy) / d(X,Y,Z)
% resxPts3d = [1,1,1];
% resyPts3d = [1,1,1];
% resxyPts3dNonrelated = zeros()

% d(reprjResxy) / d(fx,fy,cx,cy,k1,k2,p1,p2)
% resxIntrinsic = [1,0,1,0,1,1,1,1];
% resyIntrinsic = [0,1,0,1,1,1,1,1];
% derivative of reprojection residual in x respect to intrinsics index
camDresxDin = [1,3, 5:8];
camDresyDin = [2,4, 5:8];

prjDresxDin = [9,11, 13:16];
prjDresyDin = [10,12, 13:16];


% for each plane we set the Jacobian pattern entries
curRowPos = 1; % current plane's Jacobian pattern row start pos

for i = 1:numSets
    
    % current plane's # of points
    nCurPts = length(modelPts{i});
    
    % row range of current residuals x
    curRowRange = curRowPos:curRowPos+nCurPts-1;
    
    %% 1. camResx
    % d(reprjResx) / d cam(fx,fy,cx,cy,k1,k2,p1,p2)
    J(curRowRange, camDresxDin) = 1;
    
    % d(reprjResx) / d ri|ti (r1,r2,r3,t1,t2,t3), model -> cam ri|ti
    dresxdex = 23+(i-1)*nExtrinsics : 23+(i-1)*nExtrinsics+5;
    dresxdex(5) = []; % not coupled with t2 (ty)
    J(curRowRange, dresxdex) = 1;
    
    % 3d points
    % d(reprjResx) / d(X,Y,Z)
    curPts3dCol = nInExParams + pts3dPos(i);
    
    cellIdx = mat2cell(ones(1,3*nCurPts), 1, 3*ones(1,nCurPts));
    J(curRowRange, curPts3dCol:curPts3dCol+3*nCurPts-1) = sparse(blkdiag(cellIdx{:}));
    
    %% 2. camResy
    % row range of current residuals y
    curRowRange = curRowPos+nCurPts:curRowPos+2*nCurPts-1;
    
    % d(reprjResy) / d cam(fx,fy,cx,cy,k1,k2,p1,p2)
    J(curRowRange, camDresyDin) = 1;
    
    % d(reprjResx) / d ri|ti (r1,r2,r3,t1,t2,t3), model -> cam ri|ti
    dresydex = 23+(i-1)*nExtrinsics : 23+(i-1)*nExtrinsics+5;
    dresydex(4) = []; % not coupled with t1 (tx)
    J(curRowRange, dresydex) = 1;
    
    % 3d points
    % d(reprjResy) / d(X,Y,Z)
    curPts3dCol = nInExParams + pts3dPos(i);
    
    cellIdx = mat2cell(ones(1,3*nCurPts), 1, 3*ones(1,nCurPts));
    J(curRowRange, curPts3dCol:curPts3dCol+3*nCurPts-1) = sparse(blkdiag(cellIdx{:}));
    
    %% 3. prjResx
    % row range of current residuals y
    curRowRange = curRowPos+2*nCurPts:curRowPos+3*nCurPts-1;
    
    % d(reprjResx) / d prj(fx,fy,cx,cy,k1,k2,p1,p2)
    J(curRowRange, prjDresxDin) = 1;
    
    % d(reprjResx) / d ri|ti (r1,r2,r3,t1,t2,t3), model -> cam ri|ti
    dresxdex = 23+(i-1)*nExtrinsics : 23+(i-1)*nExtrinsics+5;
    dresxdex(5) = []; % not coupled with t2 (ty)
    dresxdex = [17:19,20,22,dresxdex]; % also coupled with r0|t0
    J(curRowRange, dresxdex) = 1;
    
    % 3d points
    % d(reprjResx) / d(X,Y,Z)
    curPts3dCol = nInExParams + pts3dPos(i);
    
    cellIdx = mat2cell(ones(1,3*nCurPts), 1, 3*ones(1,nCurPts));
    J(curRowRange, curPts3dCol:curPts3dCol+3*nCurPts-1) = sparse(blkdiag(cellIdx{:}));
    %% 4. prjResy
    % row range of current residuals y
    curRowRange = curRowPos+3*nCurPts:curRowPos+4*nCurPts-1;
    
    % d(reprjResy) / d prj(fx,fy,cx,cy,k1,k2,p1,p2)
    J(curRowRange, prjDresyDin) = 1;
    
    % d(reprjResy) / d ri|ti (r1,r2,r3,t1,t2,t3), model -> cam ri|ti
    dresydex = 23+(i-1)*nExtrinsics : 23+(i-1)*nExtrinsics+5;
    dresydex(4) = []; % not coupled with ti(2) (ty)
    dresydex = [17:19,21,22,dresydex]; % also coupled with r0|t0
    J(curRowRange, dresydex) = 1;
    
    % 3d points
    % d(reprjResy) / d(X,Y,Z)
    curPts3dCol = nInExParams + pts3dPos(i);
    
    cellIdx = mat2cell(ones(1,3*nCurPts), 1, 3*ones(1,nCurPts));
    J(curRowRange, curPts3dCol:curPts3dCol+3*nCurPts-1) = sparse(blkdiag(cellIdx{:}));
    
    %% current plane's Jacobian pattern row start pos
    curRowPos = curRowPos + nCurPts*4;
end

%% 5. 3dPtResXYZ
J(curRowPos:end, nInExParams+1:end) = speye(3*nPts, 3*nPts);

% if no distortion, set distortion coolumns to 0 (do not compute gradient)
if(~useDistortion)
    J(:, 5:8) = 0;   % camKc
    J(:, 13:16) = 0; % prjKc
end

%% 6. visualize
% figure;imagesc(J)
end
