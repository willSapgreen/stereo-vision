% Copyright 2011. All rights reserved.
% Institute of Measurement and Control Systems
% Karlsruhe Institute of Technology, Germany

% This file is part of libelas.
% Authors: Andreas Geiger

% libelas is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% libelas is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 

% libelas demo file for MATLAB

dbstop error; clear variables; close all;
disp('========================================');

% create figure
figure('Position',[50 100 1000 600],'Color','w');
aI = axes('Position',[0 0.1 0.5 0.9]);   axis off;
aD = axes('Position',[0.5 0.1 0.5 0.9]); axis off;
aT = axes('Position',[0.0 0.9 1.0 0.1]); axis off;

% initially show cones matching result
process(aT,aI,aD,1,'cones');

% buttons
hc = uicontrol('Style', 'checkbox', 'String', 'Subsampling', ...
               'Position', [20 60 100 30],'Value',1);

uicontrol('Style', 'pushbutton', 'String', 'Cones', ...
          'Position', [20 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''cones'');');

uicontrol('Style', 'pushbutton', 'String', 'Aloe', ...
          'Position', [120 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''aloe'');');

uicontrol('Style', 'pushbutton', 'String', 'Raindeer', ...
          'Position', [220 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''raindeer'');');
        
uicontrol('Style', 'pushbutton', 'String', 'Urban 1', ...
          'Position', [320 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''urban1'');');

uicontrol('Style', 'pushbutton', 'String', 'Urban 2', ...
          'Position', [420 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''urban2'');');

uicontrol('Style', 'pushbutton', 'String', 'Urban 3', ...
          'Position', [520 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''urban3'');');

uicontrol('Style', 'pushbutton', 'String', 'Urban 4', ...
          'Position', [620 20 80 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''),''urban4'');');

uicontrol('Style', 'pushbutton', 'String', 'Load images from files ...', ...
          'Position', [720 20 260 30], ... 
          'Callback', 'process(aT,aI,aD,get(hc,''value''));');
