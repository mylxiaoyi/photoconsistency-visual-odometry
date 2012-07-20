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

% USAGE: octave plot_odometry.m <input_estimated_poses.mat> <output_eps_file.eps> 

clear;
clc;
clf;

%Load the estimated sequence of poses (4x4 rigid transformation matrices)
load ('-ascii',argv(){1});

hold on;

xs=poses(:,4);
ys=poses(:,8);
phis=atan2(poses(:,5),poses(:,1));

%Plot pose point
plot(xs,ys,'ko');

%Plot each pose direction
lineLength = 0.05;
for(i=1:length(xs))
    plot([xs(i),xs(i)+lineLength*cos(phis(i))],
         [ys(i),ys(i)+lineLength*sin(phis(i))],'k');
end

%Plot the trajectory
plot(xs,ys,'b');

minx=min(poses(:,4));
maxx=max(poses(:,4));
miny=min(poses(:,8));
maxy=max(poses(:,8));

%Set the axis labels
xlabel('x (m)');
ylabel('y (m)');
xspace = (maxx-minx)/10;
yspace = (maxy-miny)/10;
axis([minx-xspace,maxx+yspace,miny-yspace,maxy+yspace]);
axis equal;

hold off;

print(argv(){2},'-color','-solid','-deps');
