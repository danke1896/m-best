% Code for JPDA_m Tracker 
%
% Requires:
% Point detections 
% Linear dynamic model(s), e.g. constant velocity with small acceleration 
% Gurobi ILP solver
%
% The code is tested under Windows Seven (64bit)
%
% If you use this tracker for your research, please, cite:
% S. H. Rezatofighi, A. Milan, Z. Zhang, Q. Shi, A. Dick, I. Reid, "Joint 
% Probabilistic Data Association Revisited", IEEE International Conference 
% on Computer Vision (ICCV), 2015.

% Author: S. Hamid Rezatofighi
% Last updated: Nov 17, 2015

% For questions contact the author at: hamid.rezatofighi@adelaide.edu.au


close all
clear all
clc

S_Name='S3'; % Sequence Name for PETS videos
L_Name='MF1'; % Camera name for PETS videos

AddPath; % Add the necessary paths 
PETS_Image_Detection_Path % Add the path for the image sequence and the detections 

%% Parameters 

% Parameters for Heuristics
param.Prun_Thre=0.3; % Parameter for pruning detections with the confidence score less than this value  修剪检测到的置信值小于0.3的目标
param.tret=15; % Removing tracks with life time (# frames) less than this threshold
param.Term_Frame=45; % The parameter for termination condition终止的条件

% Parameters for Kalman Filtering and JPDA
q1=0.5; % The standard deviation of the process noise for the dynamic model动态模型的过程噪声 偏离标准
qm = 7; % The standard deviation of the measurement noise测量过程的噪声偏离标准
param.Vmax=7; % maximum velocity that a target can has (used for initialization only) 目标的最高速度

param.PD=0.89; % Detection Probabilty or the average of true positive rate for detections 检测概率
param.Beta=3/(u_image*v_image); % Beta is False detection (clutter) likelihhod=Pfa/(u_image*v_image)
% Beta=(PFa Average number of false detections per frame)/(Total volume)
param.Gate=(30)^0.5; % Gate size for gating门大小？
param.S_limit=100; % parameter to stop the gate size growing too much
param.N_H=100; % Number of m-best solutions for approximating JPDA distribution 
model.JPDA_multiscale=1; % Number of processing frames for multi-frame JPDA

% Parameters for visualization
param.Plott='Yes'; % Make it 'Yes' for any visualization 
param.Box_plot='Yes'; % Make it 'Yes' to show the bounding Box for each target
param.Font=12; % Font size fot the text

%% Tracking Model 
Tracking_Models

%% Initialization
% The distribution parameters for initial state p(x_0) 
model.X0=Initialization(Detection_address,param,model); % The initial mean初始化第一帧的位置
model.P0=blkdiag([qm 0;0 1],[qm 0;0 1]); % The initial covariance初始化协方差矩阵问题：为什么是量测过程中的协方差

%% JPDA_m Tracker
[XeT,PeT,Ff,Term_Con]=JPDA_m(Detection_address,model,param,Image_address);

%% Post-processing (post processing and Removing tracks with small life spans)

X_size=cellfun(@(x) size(x,2), XeT, 'UniformOutput', false);
Ff=cellfun(@(x,y,z) x(1):x(1)+y-1-z, Ff,X_size,Term_Con, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);
Ff_size=cellfun(@(x) size(x,2), Ff, 'UniformOutput', false);
XeT=cellfun(@(x,y) x(:,1:y),XeT,Ff_size, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);
XeT2=XeT;
Ff2=Ff;
%%%%添加小轨迹跟踪代码%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
XeT_Final=TrackletAssociation(XeT2,Ff2);








% Ff(cellfun('size', XeT,2)<param.tret)=[];
% XeT(cellfun('size', XeT,2)<param.tret)=[];
% 
% 
% %% Bounding box estimation from detection
% 
% file = dir(Image_address);
% load(Detection_address) % load detection
% Frame = numel(file)-2; % Total Number of frames
% N_T=size(XeT,2);
% 
% stateInfo.X=zeros(Frame,N_T);
% stateInfo.Y=zeros(Frame,N_T);
% stateInfo.Xi=zeros(Frame,N_T);
% stateInfo.Yi=zeros(Frame,N_T);
% 
% for n=1:N_T
%     stateInfo.X(Ff{n},n)=XeT{n}(1,:);
%     stateInfo.Xi(Ff{n},n)=stateInfo.X(Ff{n},n);
%     stateInfo.Y(Ff{n},n)=XeT{n}(3,:);
%     stateInfo.Yi(Ff{n},n)=stateInfo.Y(Ff{n},n);
% end
% 
% stateInfo.frameNums=1:Frame;
% 
% if strcmp(S_Name,'S3')&&strcmp(L_Name,'MF1')
%     detections2=detections;
%     for it=1:size(detections,2)
%         detections2(it).xp=detections2(it).xi;
%         detections2(it).yp=detections2(it).yi;
%     end
% else
%     detections2=detections;
% end
% stateInfo=getBBoxesFromState(stateInfo,Ff,detections2,sceneInfo);
% 
% %% Evaluation
% 
% stateInfo2=stateInfo;
% stateInfo2.frameNums=stateInfo.frameNums-1;
% 
% % Evaluation all results using ground plan
% options.eval3d=1;   % only bounding box overlap
% options.td=1000;
% 
% [stateInfo2.Xgp,stateInfo2.Ygp]=projectToGroundPlane(stateInfo2.Xi, stateInfo2.Yi, sceneInfo);
% 
% [metrics,metricsInfo,additionalInfo]=CLEAR_MOT(gtInfo,stateInfo2,options);
% 
% printMetrics(metrics)
% 
% % Evaluation on cropped results using ground plan
% stateInfo3=stateInfo2;
% gtInfo3=gtInfo;
% 
% stateInfo3.X= stateInfo3.Xgp;
% stateInfo3.Y= stateInfo3.Ygp;
% gtInfo3.X=gtInfo3.Xgp;
% gtInfo3.Y=gtInfo3.Ygp;
% sceneInfo.trackingArea=[-14069.6, 4981.3, -14274.0, 1733.5];
% 
% 
% gtInfoC=cutGTToTrackingArea(gtInfo3, sceneInfo);
% 
% opt.track3d=1;
% stateInfoC=cutStateToTrackingArea(stateInfo3,sceneInfo, opt);
% 
% [metricsC,metricsInfoC,additionalInfoC]=CLEAR_MOT(gtInfoC,stateInfoC,struct('eval3d',1));
% 
% printMetrics(metricsC,metricsInfoC,1)
% 
% % Evaluation on cropped results using bounding box
% stNew= stateInfoC;
% gtNew=gtInfoC;
% 
% stNew.opt.track3d=0;
% 
% stNew.X=stNew.Xi;stNew.X=stNew.Yi;
% gtNew.X=gtNew.Xi;gtNew.Y=gtNew.Yi;
% 
% [metrics3d, metricsInfo3d, addInfo3d]=CLEAR_MOT(gtNew,stNew,struct('eval3d',0));
% printMetrics(metrics3d,metricsInfo3d,1);









