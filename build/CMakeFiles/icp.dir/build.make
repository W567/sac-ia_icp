# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wu/Works/ICP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wu/Works/ICP/build

# Include any dependencies generated for this target.
include CMakeFiles/icp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icp.dir/flags.make

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o: ../src/iterative_closest_point.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o -c /home/wu/Works/ICP/src/iterative_closest_point.cpp

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/iterative_closest_point.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/iterative_closest_point.cpp > CMakeFiles/icp.dir/src/iterative_closest_point.cpp.i

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/iterative_closest_point.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/iterative_closest_point.cpp -o CMakeFiles/icp.dir/src/iterative_closest_point.cpp.s

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.requires

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.provides: CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.provides

CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.provides.build: CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o


CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o: ../src/libForPCL/src/cook_geometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o -c /home/wu/Works/ICP/src/libForPCL/src/cook_geometry.cpp

CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/libForPCL/src/cook_geometry.cpp > CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.i

CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/libForPCL/src/cook_geometry.cpp -o CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.s

CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.requires

CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.provides: CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.provides

CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.provides.build: CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o


CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o: ../src/libForPCL/src/alignment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o -c /home/wu/Works/ICP/src/libForPCL/src/alignment.cpp

CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/libForPCL/src/alignment.cpp > CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.i

CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/libForPCL/src/alignment.cpp -o CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.s

CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.requires

CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.provides: CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.provides

CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.provides.build: CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o


CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o: ../src/libForPCL/src/cook_io.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o -c /home/wu/Works/ICP/src/libForPCL/src/cook_io.cpp

CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/libForPCL/src/cook_io.cpp > CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.i

CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/libForPCL/src/cook_io.cpp -o CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.s

CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.requires

CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.provides: CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.provides

CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.provides.build: CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o


CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o: ../src/libForPCL/src/cook_basis.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o -c /home/wu/Works/ICP/src/libForPCL/src/cook_basis.cpp

CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/libForPCL/src/cook_basis.cpp > CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.i

CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/libForPCL/src/cook_basis.cpp -o CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.s

CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.requires

CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.provides: CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.provides

CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.provides.build: CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o


CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o: CMakeFiles/icp.dir/flags.make
CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o: ../src/libForPCL/src/cook_seg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o -c /home/wu/Works/ICP/src/libForPCL/src/cook_seg.cpp

CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wu/Works/ICP/src/libForPCL/src/cook_seg.cpp > CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.i

CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wu/Works/ICP/src/libForPCL/src/cook_seg.cpp -o CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.s

CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.requires:

.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.requires

CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.provides: CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp.dir/build.make CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.provides.build
.PHONY : CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.provides

CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.provides.build: CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o


# Object files for target icp
icp_OBJECTS = \
"CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o" \
"CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o" \
"CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o" \
"CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o" \
"CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o" \
"CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o"

# External object files for target icp
icp_EXTERNAL_OBJECTS =

../icp: CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o
../icp: CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o
../icp: CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o
../icp: CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o
../icp: CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o
../icp: CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o
../icp: CMakeFiles/icp.dir/build.make
../icp: /usr/lib/x86_64-linux-gnu/libboost_system.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../icp: /usr/lib/x86_64-linux-gnu/libpthread.so
../icp: /usr/local/lib/libpcl_common.so
../icp: /usr/local/lib/libpcl_octree.so
../icp: /usr/local/lib/libpcl_io.so
../icp: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../icp: /usr/local/lib/libpcl_kdtree.so
../icp: /usr/local/lib/libpcl_search.so
../icp: /usr/local/lib/libpcl_sample_consensus.so
../icp: /usr/local/lib/libpcl_filters.so
../icp: /usr/local/lib/libpcl_features.so
../icp: /usr/local/lib/libpcl_keypoints.so
../icp: /usr/local/lib/libpcl_ml.so
../icp: /usr/local/lib/libpcl_segmentation.so
../icp: /usr/local/lib/libpcl_visualization.so
../icp: /usr/local/lib/libpcl_outofcore.so
../icp: /usr/local/lib/libpcl_stereo.so
../icp: /usr/lib/x86_64-linux-gnu/libqhull.so
../icp: /usr/local/lib/libpcl_surface.so
../icp: /usr/local/lib/libpcl_registration.so
../icp: /usr/local/lib/libpcl_tracking.so
../icp: /usr/local/lib/libpcl_recognition.so
../icp: /usr/local/lib/libpcl_people.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_system.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../icp: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../icp: /usr/lib/x86_64-linux-gnu/libpthread.so
../icp: /usr/lib/x86_64-linux-gnu/libqhull.so
../icp: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../icp: /usr/local/lib/libvtkIOInfovis-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingContextOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkTestingRendering-8.1.so.1
../icp: /usr/local/lib/libvtkViewsContext2D-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersGeneric-8.1.so.1
../icp: /usr/local/lib/libvtkTestingGenericBridge-8.1.so.1
../icp: /usr/local/lib/libvtkDomainsChemistryOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkIOAMR-8.1.so.1
../icp: /usr/local/lib/libvtkIOExodus-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingVolumeOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersFlowPaths-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersHyperTree-8.1.so.1
../icp: /usr/local/lib/libvtkImagingStencil-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersParallelImaging-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersPoints-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersProgrammable-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersSMP-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersSelection-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersVerdict-8.1.so.1
../icp: /usr/local/lib/libvtkIOParallel-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersTexture-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersTopology-8.1.so.1
../icp: /usr/local/lib/libvtkGeovisCore-8.1.so.1
../icp: /usr/local/lib/libvtkIOEnSight-8.1.so.1
../icp: /usr/local/lib/libvtkIOExportOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkInteractionImage-8.1.so.1
../icp: /usr/local/lib/libvtkIOImport-8.1.so.1
../icp: /usr/local/lib/libvtkIOLSDyna-8.1.so.1
../icp: /usr/local/lib/libvtkIOMINC-8.1.so.1
../icp: /usr/local/lib/libvtkIOMovie-8.1.so.1
../icp: /usr/local/lib/libvtkIOPLY-8.1.so.1
../icp: /usr/local/lib/libvtkIOParallelXML-8.1.so.1
../icp: /usr/local/lib/libvtkIOSQL-8.1.so.1
../icp: /usr/local/lib/libvtkTestingIOSQL-8.1.so.1
../icp: /usr/local/lib/libvtkIOTecplotTable-8.1.so.1
../icp: /usr/local/lib/libvtkIOVideo-8.1.so.1
../icp: /usr/local/lib/libvtkImagingStatistics-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingImage-8.1.so.1
../icp: /usr/local/lib/libvtkImagingMorphological-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingLOD-8.1.so.1
../icp: /usr/local/lib/libvtkViewsInfovis-8.1.so.1
../icp: /usr/local/lib/libpcl_common.so
../icp: /usr/local/lib/libpcl_octree.so
../icp: /usr/local/lib/libpcl_io.so
../icp: /usr/local/lib/libpcl_kdtree.so
../icp: /usr/local/lib/libpcl_search.so
../icp: /usr/local/lib/libpcl_sample_consensus.so
../icp: /usr/local/lib/libpcl_filters.so
../icp: /usr/local/lib/libpcl_features.so
../icp: /usr/local/lib/libpcl_keypoints.so
../icp: /usr/local/lib/libpcl_ml.so
../icp: /usr/local/lib/libpcl_segmentation.so
../icp: /usr/local/lib/libpcl_visualization.so
../icp: /usr/local/lib/libpcl_outofcore.so
../icp: /usr/local/lib/libpcl_stereo.so
../icp: /usr/local/lib/libpcl_surface.so
../icp: /usr/local/lib/libpcl_registration.so
../icp: /usr/local/lib/libpcl_tracking.so
../icp: /usr/local/lib/libpcl_recognition.so
../icp: /usr/local/lib/libpcl_people.so
../icp: /usr/local/lib/libvtklibxml2-8.1.so.1
../icp: /usr/local/lib/libvtkDomainsChemistry-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersAMR-8.1.so.1
../icp: /usr/local/lib/libvtkImagingMath-8.1.so.1
../icp: /usr/local/lib/libvtkverdict-8.1.so.1
../icp: /usr/local/lib/libvtkIOGeometry-8.1.so.1
../icp: /usr/local/lib/libvtkexoIIc-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersParallel-8.1.so.1
../icp: /usr/local/lib/libvtkIONetCDF-8.1.so.1
../icp: /usr/local/lib/libvtknetcdfcpp-8.1.so.1
../icp: /usr/local/lib/libvtkjsoncpp-8.1.so.1
../icp: /usr/local/lib/libvtkproj4-8.1.so.1
../icp: /usr/local/lib/libvtkIOExport-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingOpenGL2-8.1.so.1
../icp: /usr/local/lib/libvtkglew-8.1.so.1
../icp: /usr/lib/x86_64-linux-gnu/libSM.so
../icp: /usr/lib/x86_64-linux-gnu/libICE.so
../icp: /usr/lib/x86_64-linux-gnu/libX11.so
../icp: /usr/lib/x86_64-linux-gnu/libXext.so
../icp: /usr/lib/x86_64-linux-gnu/libXt.so
../icp: /usr/local/lib/libvtkgl2ps-8.1.so.1
../icp: /usr/local/lib/libvtklibharu-8.1.so.1
../icp: /usr/local/lib/libvtkNetCDF-8.1.so.1
../icp: /usr/local/lib/libvtkhdf5_hl-8.1.so.1
../icp: /usr/local/lib/libvtkhdf5-8.1.so.1
../icp: /usr/local/lib/libvtkoggtheora-8.1.so.1
../icp: /usr/local/lib/libvtkParallelCore-8.1.so.1
../icp: /usr/local/lib/libvtkIOLegacy-8.1.so.1
../icp: /usr/local/lib/libvtksqlite-8.1.so.1
../icp: /usr/local/lib/libvtkChartsCore-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingContext2D-8.1.so.1
../icp: /usr/local/lib/libvtkViewsCore-8.1.so.1
../icp: /usr/local/lib/libvtkInteractionWidgets-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersHybrid-8.1.so.1
../icp: /usr/local/lib/libvtkInteractionStyle-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingAnnotation-8.1.so.1
../icp: /usr/local/lib/libvtkImagingColor-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingVolume-8.1.so.1
../icp: /usr/local/lib/libvtkIOXML-8.1.so.1
../icp: /usr/local/lib/libvtkIOXMLParser-8.1.so.1
../icp: /usr/local/lib/libvtkIOCore-8.1.so.1
../icp: /usr/local/lib/libvtklz4-8.1.so.1
../icp: /usr/local/lib/libvtkexpat-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersImaging-8.1.so.1
../icp: /usr/local/lib/libvtkImagingGeneral-8.1.so.1
../icp: /usr/local/lib/libvtkImagingSources-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingLabel-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingFreeType-8.1.so.1
../icp: /usr/local/lib/libvtkRenderingCore-8.1.so.1
../icp: /usr/local/lib/libvtkCommonColor-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersGeometry-8.1.so.1
../icp: /usr/local/lib/libvtkfreetype-8.1.so.1
../icp: /usr/local/lib/libvtkInfovisLayout-8.1.so.1
../icp: /usr/local/lib/libvtkInfovisCore-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersExtraction-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersStatistics-8.1.so.1
../icp: /usr/local/lib/libvtkImagingFourier-8.1.so.1
../icp: /usr/local/lib/libvtkalglib-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersModeling-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersSources-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersGeneral-8.1.so.1
../icp: /usr/local/lib/libvtkCommonComputationalGeometry-8.1.so.1
../icp: /usr/local/lib/libvtkFiltersCore-8.1.so.1
../icp: /usr/local/lib/libvtkImagingHybrid-8.1.so.1
../icp: /usr/local/lib/libvtkImagingCore-8.1.so.1
../icp: /usr/local/lib/libvtkIOImage-8.1.so.1
../icp: /usr/local/lib/libvtkCommonExecutionModel-8.1.so.1
../icp: /usr/local/lib/libvtkCommonDataModel-8.1.so.1
../icp: /usr/local/lib/libvtkCommonMisc-8.1.so.1
../icp: /usr/local/lib/libvtkCommonSystem-8.1.so.1
../icp: /usr/local/lib/libvtksys-8.1.so.1
../icp: /usr/local/lib/libvtkCommonTransforms-8.1.so.1
../icp: /usr/local/lib/libvtkCommonMath-8.1.so.1
../icp: /usr/local/lib/libvtkCommonCore-8.1.so.1
../icp: /usr/local/lib/libvtkDICOMParser-8.1.so.1
../icp: /usr/local/lib/libvtkmetaio-8.1.so.1
../icp: /usr/local/lib/libvtkpng-8.1.so.1
../icp: /usr/local/lib/libvtktiff-8.1.so.1
../icp: /usr/local/lib/libvtkzlib-8.1.so.1
../icp: /usr/local/lib/libvtkjpeg-8.1.so.1
../icp: /usr/lib/x86_64-linux-gnu/libm.so
../icp: CMakeFiles/icp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wu/Works/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../icp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/icp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icp.dir/build: ../icp

.PHONY : CMakeFiles/icp.dir/build

CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/iterative_closest_point.cpp.o.requires
CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/libForPCL/src/cook_geometry.cpp.o.requires
CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/libForPCL/src/alignment.cpp.o.requires
CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/libForPCL/src/cook_io.cpp.o.requires
CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/libForPCL/src/cook_basis.cpp.o.requires
CMakeFiles/icp.dir/requires: CMakeFiles/icp.dir/src/libForPCL/src/cook_seg.cpp.o.requires

.PHONY : CMakeFiles/icp.dir/requires

CMakeFiles/icp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/icp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/icp.dir/clean

CMakeFiles/icp.dir/depend:
	cd /home/wu/Works/ICP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wu/Works/ICP /home/wu/Works/ICP /home/wu/Works/ICP/build /home/wu/Works/ICP/build /home/wu/Works/ICP/build/CMakeFiles/icp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icp.dir/depend

