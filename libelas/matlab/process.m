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

function process(aT,aI,aD,subsampling,prefix)

% load images from user input
if nargin<5
  [file_left,path_left]   = uigetfile('*.*','Select left image');
  if file_left==0, return; end
  [file_right,path_right] = uigetfile('*.*','Select right image');
  if file_left==0, return; end
  file_name = [file_left ', ' file_right];
  I1 = imread([path_left file_left]);
  I2 = imread([path_right file_right]);
  if length(size(I1))==3
    I1 = rgb2gray(I1);
  end
  if length(size(I2))==3
    I2 = rgb2gray(I2);
  end
  
% load images from prefix
else
  file_name = [prefix '_left.pgm, ' prefix '_right.pgm'];
  I1 = imread(['../img/' prefix '_left.pgm']);
  I2 = imread(['../img/' prefix '_right.pgm']);
end

% create title
axes(aT); cla;
title_text = ['Image pair: ''' file_name '''  /  Size: ' num2str(size(I1,1)*size(I1,2)*1e-6,'%2.2f') ' MPx'];
text(0.5,0.5,title_text,'HorizontalAlignment','center','FontSize',16,'Interpreter','none');

% show images
axes(aI); cla; imshow_rgb(I1);
axes(aD); cla; refresh; pause(0.1);

% matching parameters (see elas.h for a full list)
% if you want to reproduce the middlebury results, disable subsampling,
% go to elasMex.cpp and decomment Elas::parameters param(Elas::MIDDLEBURY);

param.disp_min    = 0;           % minimum disparity (positive integer)
param.disp_max    = 255;         % maximum disparity (positive integer)
param.subsampling = subsampling; % process only each 2nd pixel (1=active)

% perform actual matching
try
  tic;
  [D1,D2] = elasMex(I1',I2',param);
  time = toc;  
catch me
  disp(me.message);
  axes(aT);  cla;
  title_text = 'ERROR: Compile MATLAB wrappers by running ''make''';
  disp(title_text);
  text(0.5,0.5,title_text,'HorizontalAlignment','center','FontSize',16,'Interpreter','none');
  return;
end

% update title with elapsed time
axes(aT); cla;
title_text = [title_text '  /  Elapsed time: ' num2str(time,'%2.2f') ' sec'];
text(0.5,0.5,title_text,'HorizontalAlignment','center','FontSize',16,'Interpreter','none');

%D1(D1>50)=-1;
%keyboard;

% show matching results
axes(aD); imagesc(D1'); colormap(jet(256));
axis off; axis equal;
refresh; pause(0.1);

function imshow_rgb(I)

I_rgb = zeros(size(I,1),size(I,2),3,'uint8');
I_rgb(:,:,1) = I;
I_rgb(:,:,2) = I;
I_rgb(:,:,3) = I;
imshow(I_rgb);

