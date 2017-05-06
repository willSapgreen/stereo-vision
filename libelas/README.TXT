####################################################################################
# Copyright 2011. All rights reserved.                                             #
# Institute of Measurement and Control Systems                                     #
# Karlsruhe Institute of Technology, Germany                                       #
#                                                                                  #
# This file is part of libelas.                                                    #
# Authors:  Andreas Geiger                                                         #
#           Please send any bugreports to geiger@kit.edu                           #
#                                                                                  #
# libelas is free software; you can redistribute it and/or modify it under the     #
# terms of the GNU General Public License as published by the Free Software        #
# Foundation; either version 3 of the License, or any later version.               #
#                                                                                  #
# libelas is distributed in the hope that it will be useful, but WITHOUT ANY       #
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A  #
# PARTICULAR PURPOSE. See the GNU General Public License for more details.         #
#                                                                                  #
# You should have received a copy of the GNU General Public License along with     #
# libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin        #
# Street, Fifth Floor, Boston, MA 02110-1301, USA                                  #
####################################################################################

+++++++++++++++++++++++++++++++++++
+          INTRODUCTION           +
+++++++++++++++++++++++++++++++++++

Libelas (LIBrary for Efficient LArge-scale Stereo matching) is a cross-platfrom C++
library with MATLAB wrappers for computing disparity maps of large images. Input
is a rectified greyscale stereo image pair of same size. Output are the corresponding
disparity maps.

If you distribute a software that uses libelas, you have to distribute it under GPL
with the source code. Another option is to contact us to purchase a commercial license.

If you find this software useful or if you use this software for your research,
we would be happy if you cite the following related publication:

@INPROCEEDINGS{Geiger10,
 author = {Andreas Geiger and Martin Roser and Raquel Urtasun},
 title = {Efficient Large-Scale Stereo Matching},
 booktitle = {Asian Conference on Computer Vision},
 year = {2010},
 month = {November},
 address = {Queenstown, New Zealand}
}

+++++++++++++++++++++++++++++++++++
+    COMPILING MATLAB WRAPPERS    +
+++++++++++++++++++++++++++++++++++

If you want to use libelas directly from MATLAB you can easily do this by using
the MATLAB wrappers provided. They also include some demo files for testing your
configuration. First, configure your MATLAB MEX C++ compiler, if it is not yet
configured (mex -setup). Under Linux you might use g++, under Windows I compiled
it successfully with the Microsoft Visual Studio Express 2008 compilers.

1) Change to the libelas/matlab directory
2) After running 'make.m' you should have a MEX file called 'elasMex'
3) Now try to run 'demo.m' which opens a GUI and shows you
   some results on the included test images

+++++++++++++++++++++++++++++++++++
+     BUILDING A C++ LIBRARY      +
+++++++++++++++++++++++++++++++++++

Prerequisites needed for compiling libelas using c++:
- CMake (available at: http://www.cmake.org/)

Linux:

1) Move to libelas root directory
2) Type 'cmake .'
3) Type 'make'
4) Run './elas demo' => computes disparity maps for images from the 'img' directory

Windows:

1) Start CMake GUI
2) Set directories to elas root directory
3) Run configure, configure and generate
4) Open the resulting Visual Studio solution with Visual Studio
5) Switch to 'Release' mode and build all
6) Move 'elas.exe' from libelas/Release to libelas
7) Open a console and navigate to libelas root directory
8) Run 'elas.exe demo' => computes disparity maps in the img directory

For more information on CMake, have a look at the CMake documentation.

For more information on the usage of the library, have a look into the MATLAB wrappers and
into the documentation of the header elas.h.

Please send any feedback and bugreports to geiger@kit.edu
Andreas Geiger

