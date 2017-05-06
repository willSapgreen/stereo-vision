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

% MATLAB makefile for compiling the libelas MATLAB wrappers

disp('Building wrappers ...');

% standard version
mex('elasMex.cpp','../src/elas.cpp','../src/descriptor.cpp', '../src/filter.cpp', ...
    '../src/triangle.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=\$CXXFLAGS -msse3 -fPIC');

% version for profiling individual timings (only for linux)
%mex('elasMex.cpp','../src/elas.cpp','../src/descriptor.cpp', ...
%    '../src/triangle.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=\$CXXFLAGS -msse3 -fPIC','-DPROFILE');

disp('...done!');
