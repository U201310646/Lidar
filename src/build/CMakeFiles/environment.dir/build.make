# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cyr/Documents/cppProject/LidarProjects/Lidar/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build

# Include any dependencies generated for this target.
include CMakeFiles/environment.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/environment.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/environment.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/environment.dir/flags.make

CMakeFiles/environment.dir/box.cpp.o: CMakeFiles/environment.dir/flags.make
CMakeFiles/environment.dir/box.cpp.o: ../box.cpp
CMakeFiles/environment.dir/box.cpp.o: CMakeFiles/environment.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/environment.dir/box.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/environment.dir/box.cpp.o -MF CMakeFiles/environment.dir/box.cpp.o.d -o CMakeFiles/environment.dir/box.cpp.o -c /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/box.cpp

CMakeFiles/environment.dir/box.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/environment.dir/box.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/box.cpp > CMakeFiles/environment.dir/box.cpp.i

CMakeFiles/environment.dir/box.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/environment.dir/box.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/box.cpp -o CMakeFiles/environment.dir/box.cpp.s

CMakeFiles/environment.dir/render.cpp.o: CMakeFiles/environment.dir/flags.make
CMakeFiles/environment.dir/render.cpp.o: ../render.cpp
CMakeFiles/environment.dir/render.cpp.o: CMakeFiles/environment.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/environment.dir/render.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/environment.dir/render.cpp.o -MF CMakeFiles/environment.dir/render.cpp.o.d -o CMakeFiles/environment.dir/render.cpp.o -c /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/render.cpp

CMakeFiles/environment.dir/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/environment.dir/render.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/render.cpp > CMakeFiles/environment.dir/render.cpp.i

CMakeFiles/environment.dir/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/environment.dir/render.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/render.cpp -o CMakeFiles/environment.dir/render.cpp.s

CMakeFiles/environment.dir/environment.cpp.o: CMakeFiles/environment.dir/flags.make
CMakeFiles/environment.dir/environment.cpp.o: ../environment.cpp
CMakeFiles/environment.dir/environment.cpp.o: CMakeFiles/environment.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/environment.dir/environment.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/environment.dir/environment.cpp.o -MF CMakeFiles/environment.dir/environment.cpp.o.d -o CMakeFiles/environment.dir/environment.cpp.o -c /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/environment.cpp

CMakeFiles/environment.dir/environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/environment.dir/environment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/environment.cpp > CMakeFiles/environment.dir/environment.cpp.i

CMakeFiles/environment.dir/environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/environment.dir/environment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/environment.cpp -o CMakeFiles/environment.dir/environment.cpp.s

# Object files for target environment
environment_OBJECTS = \
"CMakeFiles/environment.dir/box.cpp.o" \
"CMakeFiles/environment.dir/render.cpp.o" \
"CMakeFiles/environment.dir/environment.cpp.o"

# External object files for target environment
environment_EXTERNAL_OBJECTS =

environment: CMakeFiles/environment.dir/box.cpp.o
environment: CMakeFiles/environment.dir/render.cpp.o
environment: CMakeFiles/environment.dir/environment.cpp.o
environment: CMakeFiles/environment.dir/build.make
environment: /usr/local/lib/libpcl_surface.so
environment: /usr/local/lib/libpcl_keypoints.so
environment: /usr/local/lib/libpcl_tracking.so
environment: /usr/local/lib/libpcl_recognition.so
environment: /usr/local/lib/libpcl_stereo.so
environment: /usr/local/lib/libpcl_outofcore.so
environment: /usr/local/lib/libpcl_people.so
environment: /usr/lib/x86_64-linux-gnu/libboost_system.so
environment: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
environment: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
environment: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
environment: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
environment: /usr/lib/x86_64-linux-gnu/libboost_regex.so
environment: /usr/lib/x86_64-linux-gnu/libqhull.so
environment: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libfreetype.so
environment: /usr/lib/x86_64-linux-gnu/libz.so
environment: /usr/lib/x86_64-linux-gnu/libjpeg.so
environment: /usr/lib/x86_64-linux-gnu/libpng.so
environment: /usr/lib/x86_64-linux-gnu/libtiff.so
environment: /usr/lib/x86_64-linux-gnu/libexpat.so
environment: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
environment: /usr/local/lib/libpcl_registration.so
environment: /usr/local/lib/libpcl_segmentation.so
environment: /usr/local/lib/libpcl_features.so
environment: /usr/local/lib/libpcl_filters.so
environment: /usr/local/lib/libpcl_sample_consensus.so
environment: /usr/local/lib/libpcl_ml.so
environment: /usr/local/lib/libpcl_visualization.so
environment: /usr/local/lib/libpcl_search.so
environment: /usr/local/lib/libpcl_kdtree.so
environment: /usr/local/lib/libpcl_io.so
environment: /usr/local/lib/libpcl_octree.so
environment: /usr/local/lib/libpcl_common.so
environment: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libfreetype.so
environment: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
environment: /usr/lib/x86_64-linux-gnu/libz.so
environment: /usr/lib/x86_64-linux-gnu/libGLEW.so
environment: /usr/lib/x86_64-linux-gnu/libSM.so
environment: /usr/lib/x86_64-linux-gnu/libICE.so
environment: /usr/lib/x86_64-linux-gnu/libX11.so
environment: /usr/lib/x86_64-linux-gnu/libXext.so
environment: /usr/lib/x86_64-linux-gnu/libXt.so
environment: CMakeFiles/environment.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable environment"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/environment.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/environment.dir/build: environment
.PHONY : CMakeFiles/environment.dir/build

CMakeFiles/environment.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/environment.dir/cmake_clean.cmake
.PHONY : CMakeFiles/environment.dir/clean

CMakeFiles/environment.dir/depend:
	cd /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cyr/Documents/cppProject/LidarProjects/Lidar/src /home/cyr/Documents/cppProject/LidarProjects/Lidar/src /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build /home/cyr/Documents/cppProject/LidarProjects/Lidar/src/build/CMakeFiles/environment.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/environment.dir/depend

