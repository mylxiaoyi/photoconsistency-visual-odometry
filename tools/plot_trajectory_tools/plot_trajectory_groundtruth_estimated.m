#!/usr/bin/env octave -q

 %  Photoconsistency-Visual-Odometry
 %  Multiscale Photoconsistency Visual Odometry from RGBD Images
 %  Copyright (c) 2012, Miguel Algaba Borrego
 %  
 %  http://code.google.com/p/photoconsistency-visual-odometry/
 %  
 %  All rights reserved.
 %  
 %  Redistribution and use in source and binary forms, with or without
 %  modification, are permitted provided that the following conditions are met:
 %      * Redistributions of source code must retain the above copyright
 %        notice, this list of conditions and the following disclaimer.
 %      * Redistributions in binary form must reproduce the above copyright
 %        notice, this list of conditions and the following disclaimer in the
 %        documentation and/or other materials provided with the distribution.
 %      * Neither the name of the holder(s) nor the
 %        names of its contributors may be used to endorse or promote products
 %        derived from this software without specific prior written permission.
 %  
 %  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 %  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 %  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 %  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 %  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 %  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 %  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 %  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 %  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 %  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% USAGE: octave plot_trajectory_groundtruth_estimated.m <input_estimated_trajectory.txt> <input_groundtruth.txt> <output_eps_file.eps> 

clear;
clc;
clf;

hold on;

%Load the estimated trajectory (3D translations + Quaternions)
load ('-ascii',argv(){1});

%Load the ground truth (3D translations + Quaternions)
load ('-ascii',argv(){2});

%Remove all the ground-truth measurements taken before and after the rgbd dataset
firstTime = trajectory(1,1);
lastTime = trajectory(end,1);
groundtruth = groundtruth(groundtruth(:,1)>=firstTime & groundtruth(:,1)<=lastTime,:);

%Compute the translation and rotation components of the ground-truth(for the 3DoF case)
txs=groundtruth(:,2);
tys=groundtruth(:,3);
tzs=groundtruth(:,4);

%Compute the translation and rotation components of the estimated trajectory (for the 3DoF case)
txs_estimate=trajectory(:,2);
tys_estimate=trajectory(:,3);
tzs_estimate=trajectory(:,4);
qxs_estimate=trajectory(:,5);
qys_estimate=trajectory(:,6);
qzs_estimate=trajectory(:,7);
qws_estimate=trajectory(:,8);
qws_estimate_square=qws_estimate.^2;
qxs_estimate_square=qxs_estimate.^2;
qys_estimate_square=qys_estimate.^2;
qzs_estimate_square=qzs_estimate.^2;
posesEstimatedTrajectory_1_1 = qws_estimate_square+qxs_estimate_square-qys_estimate_square-qzs_estimate_square;
posesEstimatedTrajectory_2_1 = 2*(qxs_estimate.*qys_estimate+qws_estimate.*qzs_estimate);
phis_estimate = atan2(posesEstimatedTrajectory_2_1,posesEstimatedTrajectory_1_1).+pi/2;

%Compute rigid transform between ground-truth and estimated trajectory
%Obtain auxiliary 3D points from the ground-truth that correspond with the 3D points in the trajectory.
txs_aux=zeros(length(txs_estimate)-1,1);
tys_aux=zeros(length(tys_estimate)-1,1);
tzs_aux=zeros(length(tzs_estimate)-1,1);
for(i=1:length(trajectory)-1)
    pointIndex=find(groundtruth(:,1)>=trajectory(i,1),1,'first');
    txs_aux(i,1)=groundtruth(pointIndex,2);
    tys_aux(i,1)=groundtruth(pointIndex,3);	
    tzs_aux(i,1)=groundtruth(pointIndex,4);
end

Rt=rigid_transform_3D([txs_aux,tys_aux,tzs_aux],[txs_estimate(1:end-1),tys_estimate(1:end-1),tzs_estimate(1:end-1)]);

%Plot the trajectory [ground-truth]
for(i=1:length(groundtruth))
	groundtruthTransformed(i,:,:)=Rt*[txs(i);tys(i);tzs(i);1];
end
plot(groundtruthTransformed(:,1),groundtruthTransformed(:,2),'k');

%Plot pose point
plot(txs_estimate,tys_estimate,'bo');

%Plot each pose direction
lineLength = 0.05;
for(i=1:length(txs_estimate))
    plot([txs_estimate(i),txs_estimate(i)+lineLength*cos(phis_estimate(i))],
         [tys_estimate(i),tys_estimate(i)+lineLength*sin(phis_estimate(i))],'b');
end

%Plot the trajectory
plot(txs_estimate,tys_estimate,'b');

%Set the axis labels
xlabel('x (m)');
ylabel('y (m)');
axis equal;
legend('Ground truth','','','Estimated trajectory');

hold off;

print(argv(){3},'-color','-solid','-deps');

