# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/agilex/Desktop/Software/task1/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/agilex/Desktop/Software/task1/build

# Include any dependencies generated for this target.
include task1/CMakeFiles/task1_4.dir/depend.make

# Include the progress variables for this target.
include task1/CMakeFiles/task1_4.dir/progress.make

# Include the compile flags for this target's objects.
include task1/CMakeFiles/task1_4.dir/flags.make

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o: task1/CMakeFiles/task1_4.dir/flags.make
task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o: /home/agilex/Desktop/Software/task1/src/task1/src/exp4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/agilex/Desktop/Software/task1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o"
	cd /home/agilex/Desktop/Software/task1/build/task1 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task1_4.dir/src/exp4.cpp.o -c /home/agilex/Desktop/Software/task1/src/task1/src/exp4.cpp

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task1_4.dir/src/exp4.cpp.i"
	cd /home/agilex/Desktop/Software/task1/build/task1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/agilex/Desktop/Software/task1/src/task1/src/exp4.cpp > CMakeFiles/task1_4.dir/src/exp4.cpp.i

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task1_4.dir/src/exp4.cpp.s"
	cd /home/agilex/Desktop/Software/task1/build/task1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/agilex/Desktop/Software/task1/src/task1/src/exp4.cpp -o CMakeFiles/task1_4.dir/src/exp4.cpp.s

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.requires:

.PHONY : task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.requires

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.provides: task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.requires
	$(MAKE) -f task1/CMakeFiles/task1_4.dir/build.make task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.provides.build
.PHONY : task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.provides

task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.provides.build: task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o


# Object files for target task1_4
task1_4_OBJECTS = \
"CMakeFiles/task1_4.dir/src/exp4.cpp.o"

# External object files for target task1_4
task1_4_EXTERNAL_OBJECTS =

/home/agilex/Desktop/Software/task1/src/task1/task1_4: task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o
/home/agilex/Desktop/Software/task1/src/task1/task1_4: task1/CMakeFiles/task1_4.dir/build.make
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libimage_transport.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libmessage_filters.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libclass_loader.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/libPocoFoundation.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libroslib.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/librospack.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /home/agilex/catkin_ws/devel/lib/libcv_bridge.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpcl_common.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpcl_octree.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpcl_io.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_serialization.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/libOpenNI.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/libOpenNI2.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtksys-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkalglib-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libfreetype.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libz.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkftgl-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOXML-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libexpat.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/libvtkWrappingTools-6.3.a
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkverdict-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libjpeg.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpng.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libtiff.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libsqlite3.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libproj.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/hdf5/openmpi/libhdf5.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libsz.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libdl.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libm.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libnetcdf_c++.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libnetcdf.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOExport-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libgl2ps.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libtheoraenc.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libtheoradec.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libogg.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOImport-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libxml2.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libjsoncpp.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkVPIC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libroscpp.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/librosconsole.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/librostime.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /opt/ros/melodic/lib/libcpp_common.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/agilex/Desktop/Software/task1/src/task1/task1_4: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/agilex/Desktop/Software/task1/src/task1/task1_4: task1/CMakeFiles/task1_4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/agilex/Desktop/Software/task1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/agilex/Desktop/Software/task1/src/task1/task1_4"
	cd /home/agilex/Desktop/Software/task1/build/task1 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/task1_4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
task1/CMakeFiles/task1_4.dir/build: /home/agilex/Desktop/Software/task1/src/task1/task1_4

.PHONY : task1/CMakeFiles/task1_4.dir/build

task1/CMakeFiles/task1_4.dir/requires: task1/CMakeFiles/task1_4.dir/src/exp4.cpp.o.requires

.PHONY : task1/CMakeFiles/task1_4.dir/requires

task1/CMakeFiles/task1_4.dir/clean:
	cd /home/agilex/Desktop/Software/task1/build/task1 && $(CMAKE_COMMAND) -P CMakeFiles/task1_4.dir/cmake_clean.cmake
.PHONY : task1/CMakeFiles/task1_4.dir/clean

task1/CMakeFiles/task1_4.dir/depend:
	cd /home/agilex/Desktop/Software/task1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/agilex/Desktop/Software/task1/src /home/agilex/Desktop/Software/task1/src/task1 /home/agilex/Desktop/Software/task1/build /home/agilex/Desktop/Software/task1/build/task1 /home/agilex/Desktop/Software/task1/build/task1/CMakeFiles/task1_4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : task1/CMakeFiles/task1_4.dir/depend

