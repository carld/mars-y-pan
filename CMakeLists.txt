#
# Copyright (c) 2008-2016 the Urho3D project.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
cmake_policy(SET CMP0015 OLD) 

project(Mars)

cmake_minimum_required (VERSION 3.2.3)

set (CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/Modules)
MESSAGE("${CMAKE_MODULE_PATH}") 

include (Urho3D-CMake-common)
find_package (Urho3D REQUIRED)
include_directories(${URHO3D_HOME}/include/Urho3D/ThirdParty/Lua${JIT})
include_directories(${URHO3D_HOME}/include/Urho3D/ThirdParty/toluapp)
include_directories(${URHO3D_HOME}/include/Urho3D/ThirdParty/AngelScript)
include_directories (${URHO3D_INCLUDE_DIRS})
include_directories (${INCLUDE_DIRS_ONLY})

# Define target name
set (TARGET_NAME Mars)

# Define source files
define_source_files () #EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES})

setup_main_executable ()

# Setup test cases
#setup_test ()
