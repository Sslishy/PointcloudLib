# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/slishy/Code/class/lib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/slishy/Code/class/lib/build

# Include any dependencies generated for this target.
include CMakeFiles/ThreepointsV2R.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ThreepointsV2R.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ThreepointsV2R.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ThreepointsV2R.dir/flags.make

CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o: CMakeFiles/ThreepointsV2R.dir/flags.make
CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o: ../ThreepointsV2R.cpp
CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o: CMakeFiles/ThreepointsV2R.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/slishy/Code/class/lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o -MF CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o.d -o CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o -c /home/slishy/Code/class/lib/ThreepointsV2R.cpp

CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.i"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/slishy/Code/class/lib/ThreepointsV2R.cpp > CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.i

CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.s"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/slishy/Code/class/lib/ThreepointsV2R.cpp -o CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.s

# Object files for target ThreepointsV2R
ThreepointsV2R_OBJECTS = \
"CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o"

# External object files for target ThreepointsV2R
ThreepointsV2R_EXTERNAL_OBJECTS =

ThreepointsV2R: CMakeFiles/ThreepointsV2R.dir/ThreepointsV2R.cpp.o
ThreepointsV2R: CMakeFiles/ThreepointsV2R.dir/build.make
ThreepointsV2R: libPlanefitting.so
ThreepointsV2R: libComputepointspose.so
ThreepointsV2R: libcomputeangle.so
ThreepointsV2R: libCollisionDetection.so
ThreepointsV2R: libPointviewer.so
ThreepointsV2R: libcylinderfitting.so
ThreepointsV2R: libPointProcess.so
ThreepointsV2R: libPointCloudAligment.so
ThreepointsV2R: /usr/local/lib/libpcl_surface.so
ThreepointsV2R: /usr/local/lib/libpcl_keypoints.so
ThreepointsV2R: /usr/local/lib/libpcl_tracking.so
ThreepointsV2R: /usr/local/lib/libpcl_recognition.so
ThreepointsV2R: /usr/local/lib/libpcl_stereo.so
ThreepointsV2R: /usr/local/lib/libpcl_outofcore.so
ThreepointsV2R: /usr/local/lib/libpcl_people.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_system.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libboost_regex.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libqhull.so
ThreepointsV2R: /usr/lib/libOpenNI.so
ThreepointsV2R: /usr/lib/libOpenNI2.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libfreetype.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libz.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libjpeg.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libpng.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libtiff.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libexpat.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
ThreepointsV2R: /usr/local/lib/libpcl_registration.so
ThreepointsV2R: /usr/local/lib/libpcl_segmentation.so
ThreepointsV2R: /usr/local/lib/libpcl_features.so
ThreepointsV2R: /usr/local/lib/libpcl_filters.so
ThreepointsV2R: /usr/local/lib/libpcl_sample_consensus.so
ThreepointsV2R: /usr/local/lib/libpcl_ml.so
ThreepointsV2R: /usr/local/lib/libpcl_visualization.so
ThreepointsV2R: /usr/local/lib/libpcl_search.so
ThreepointsV2R: /usr/local/lib/libpcl_kdtree.so
ThreepointsV2R: /usr/local/lib/libpcl_io.so
ThreepointsV2R: /usr/local/lib/libpcl_octree.so
ThreepointsV2R: /usr/local/lib/libpcl_common.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libz.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libGLEW.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libSM.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libICE.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libX11.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libXext.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libXt.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libfreetype.so
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
ThreepointsV2R: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
ThreepointsV2R: CMakeFiles/ThreepointsV2R.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/slishy/Code/class/lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ThreepointsV2R"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ThreepointsV2R.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ThreepointsV2R.dir/build: ThreepointsV2R
.PHONY : CMakeFiles/ThreepointsV2R.dir/build

CMakeFiles/ThreepointsV2R.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ThreepointsV2R.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ThreepointsV2R.dir/clean

CMakeFiles/ThreepointsV2R.dir/depend:
	cd /home/slishy/Code/class/lib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/slishy/Code/class/lib /home/slishy/Code/class/lib /home/slishy/Code/class/lib/build /home/slishy/Code/class/lib/build /home/slishy/Code/class/lib/build/CMakeFiles/ThreepointsV2R.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ThreepointsV2R.dir/depend

