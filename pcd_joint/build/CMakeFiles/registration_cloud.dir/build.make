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
CMAKE_SOURCE_DIR = /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build

# Include any dependencies generated for this target.
include CMakeFiles/registration_cloud.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/registration_cloud.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/registration_cloud.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/registration_cloud.dir/flags.make

CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o: CMakeFiles/registration_cloud.dir/flags.make
CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o: ../registration_cloud.cpp
CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o: CMakeFiles/registration_cloud.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o -MF CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o.d -o CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o -c /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/registration_cloud.cpp

CMakeFiles/registration_cloud.dir/registration_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/registration_cloud.dir/registration_cloud.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/registration_cloud.cpp > CMakeFiles/registration_cloud.dir/registration_cloud.cpp.i

CMakeFiles/registration_cloud.dir/registration_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/registration_cloud.dir/registration_cloud.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/registration_cloud.cpp -o CMakeFiles/registration_cloud.dir/registration_cloud.cpp.s

# Object files for target registration_cloud
registration_cloud_OBJECTS = \
"CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o"

# External object files for target registration_cloud
registration_cloud_EXTERNAL_OBJECTS =

registration_cloud: CMakeFiles/registration_cloud.dir/registration_cloud.cpp.o
registration_cloud: CMakeFiles/registration_cloud.dir/build.make
registration_cloud: /usr/local/lib/libpcl_surface.so
registration_cloud: /usr/local/lib/libpcl_keypoints.so
registration_cloud: /usr/local/lib/libpcl_tracking.so
registration_cloud: /usr/local/lib/libpcl_recognition.so
registration_cloud: /usr/local/lib/libpcl_stereo.so
registration_cloud: /usr/local/lib/libpcl_outofcore.so
registration_cloud: /usr/local/lib/libpcl_people.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libqhull.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libfreetype.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libz.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libjpeg.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libpng.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libtiff.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libexpat.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
registration_cloud: /usr/local/lib/libpcl_registration.so
registration_cloud: /usr/local/lib/libpcl_segmentation.so
registration_cloud: /usr/local/lib/libpcl_features.so
registration_cloud: /usr/local/lib/libpcl_filters.so
registration_cloud: /usr/local/lib/libpcl_sample_consensus.so
registration_cloud: /usr/local/lib/libpcl_ml.so
registration_cloud: /usr/local/lib/libpcl_visualization.so
registration_cloud: /usr/local/lib/libpcl_search.so
registration_cloud: /usr/local/lib/libpcl_kdtree.so
registration_cloud: /usr/local/lib/libpcl_io.so
registration_cloud: /usr/local/lib/libpcl_octree.so
registration_cloud: /usr/local/lib/libpcl_common.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libfreetype.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
registration_cloud: /usr/lib/x86_64-linux-gnu/libz.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libGLEW.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libSM.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libICE.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libX11.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libXext.so
registration_cloud: /usr/lib/x86_64-linux-gnu/libXt.so
registration_cloud: CMakeFiles/registration_cloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable registration_cloud"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/registration_cloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/registration_cloud.dir/build: registration_cloud
.PHONY : CMakeFiles/registration_cloud.dir/build

CMakeFiles/registration_cloud.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/registration_cloud.dir/cmake_clean.cmake
.PHONY : CMakeFiles/registration_cloud.dir/clean

CMakeFiles/registration_cloud.dir/depend:
	cd /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build /home/cyr/Documents/cppProject/LidarProjects/Lidar/pcd_joint/build/CMakeFiles/registration_cloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/registration_cloud.dir/depend
