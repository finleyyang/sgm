# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /home/finley/Downloads/clion-2021.2.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/finley/Downloads/clion-2021.2.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/finley/CODE/sgm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/finley/CODE/sgm/build

# Include any dependencies generated for this target.
include CMakeFiles/sgm.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/sgm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sgm.dir/flags.make

CMakeFiles/sgm.dir/main.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sgm.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/main.cpp.o -c /home/finley/CODE/sgm/main.cpp

CMakeFiles/sgm.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/main.cpp > CMakeFiles/sgm.dir/main.cpp.i

CMakeFiles/sgm.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/main.cpp -o CMakeFiles/sgm.dir/main.cpp.s

CMakeFiles/sgm.dir/Camera.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/Camera.cpp.o: ../Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sgm.dir/Camera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/Camera.cpp.o -c /home/finley/CODE/sgm/Camera.cpp

CMakeFiles/sgm.dir/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/Camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/Camera.cpp > CMakeFiles/sgm.dir/Camera.cpp.i

CMakeFiles/sgm.dir/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/Camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/Camera.cpp -o CMakeFiles/sgm.dir/Camera.cpp.s

CMakeFiles/sgm.dir/Image.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/Image.cpp.o: ../Image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/sgm.dir/Image.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/Image.cpp.o -c /home/finley/CODE/sgm/Image.cpp

CMakeFiles/sgm.dir/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/Image.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/Image.cpp > CMakeFiles/sgm.dir/Image.cpp.i

CMakeFiles/sgm.dir/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/Image.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/Image.cpp -o CMakeFiles/sgm.dir/Image.cpp.s

CMakeFiles/sgm.dir/sgm.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/sgm.cpp.o: ../sgm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/sgm.dir/sgm.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/sgm.cpp.o -c /home/finley/CODE/sgm/sgm.cpp

CMakeFiles/sgm.dir/sgm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/sgm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/sgm.cpp > CMakeFiles/sgm.dir/sgm.cpp.i

CMakeFiles/sgm.dir/sgm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/sgm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/sgm.cpp -o CMakeFiles/sgm.dir/sgm.cpp.s

CMakeFiles/sgm.dir/Plane.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/Plane.cpp.o: ../Plane.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/sgm.dir/Plane.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/Plane.cpp.o -c /home/finley/CODE/sgm/Plane.cpp

CMakeFiles/sgm.dir/Plane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/Plane.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/Plane.cpp > CMakeFiles/sgm.dir/Plane.cpp.i

CMakeFiles/sgm.dir/Plane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/Plane.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/Plane.cpp -o CMakeFiles/sgm.dir/Plane.cpp.s

CMakeFiles/sgm.dir/Ray.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/Ray.cpp.o: ../Ray.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/sgm.dir/Ray.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/Ray.cpp.o -c /home/finley/CODE/sgm/Ray.cpp

CMakeFiles/sgm.dir/Ray.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/Ray.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/Ray.cpp > CMakeFiles/sgm.dir/Ray.cpp.i

CMakeFiles/sgm.dir/Ray.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/Ray.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/Ray.cpp -o CMakeFiles/sgm.dir/Ray.cpp.s

CMakeFiles/sgm.dir/common.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/common.cpp.o: ../common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/sgm.dir/common.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/common.cpp.o -c /home/finley/CODE/sgm/common.cpp

CMakeFiles/sgm.dir/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finley/CODE/sgm/common.cpp > CMakeFiles/sgm.dir/common.cpp.i

CMakeFiles/sgm.dir/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finley/CODE/sgm/common.cpp -o CMakeFiles/sgm.dir/common.cpp.s

# Object files for target sgm
sgm_OBJECTS = \
"CMakeFiles/sgm.dir/main.cpp.o" \
"CMakeFiles/sgm.dir/Camera.cpp.o" \
"CMakeFiles/sgm.dir/Image.cpp.o" \
"CMakeFiles/sgm.dir/sgm.cpp.o" \
"CMakeFiles/sgm.dir/Plane.cpp.o" \
"CMakeFiles/sgm.dir/Ray.cpp.o" \
"CMakeFiles/sgm.dir/common.cpp.o"

# External object files for target sgm
sgm_EXTERNAL_OBJECTS =

sgm: CMakeFiles/sgm.dir/main.cpp.o
sgm: CMakeFiles/sgm.dir/Camera.cpp.o
sgm: CMakeFiles/sgm.dir/Image.cpp.o
sgm: CMakeFiles/sgm.dir/sgm.cpp.o
sgm: CMakeFiles/sgm.dir/Plane.cpp.o
sgm: CMakeFiles/sgm.dir/Ray.cpp.o
sgm: CMakeFiles/sgm.dir/common.cpp.o
sgm: CMakeFiles/sgm.dir/build.make
sgm: /usr/lib/x86_64-linux-gnu/libmpfr.so
sgm: /usr/lib/x86_64-linux-gnu/libgmpxx.so
sgm: /usr/lib/x86_64-linux-gnu/libgmp.so
sgm: /usr/lib/x86_64-linux-gnu/libCGAL.so.13.0.1
sgm: /usr/local/lib/libopencv_gapi.so.4.5.4
sgm: /usr/local/lib/libopencv_highgui.so.4.5.4
sgm: /usr/local/lib/libopencv_ml.so.4.5.4
sgm: /usr/local/lib/libopencv_objdetect.so.4.5.4
sgm: /usr/local/lib/libopencv_photo.so.4.5.4
sgm: /usr/local/lib/libopencv_stitching.so.4.5.4
sgm: /usr/local/lib/libopencv_video.so.4.5.4
sgm: /usr/local/lib/libopencv_videoio.so.4.5.4
sgm: /usr/local/lib/libopencv_imgcodecs.so.4.5.4
sgm: /usr/local/lib/libopencv_dnn.so.4.5.4
sgm: /usr/local/lib/libopencv_calib3d.so.4.5.4
sgm: /usr/local/lib/libopencv_features2d.so.4.5.4
sgm: /usr/local/lib/libopencv_flann.so.4.5.4
sgm: /usr/local/lib/libopencv_imgproc.so.4.5.4
sgm: /usr/local/lib/libopencv_core.so.4.5.4
sgm: CMakeFiles/sgm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/finley/CODE/sgm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable sgm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sgm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sgm.dir/build: sgm
.PHONY : CMakeFiles/sgm.dir/build

CMakeFiles/sgm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sgm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sgm.dir/clean

CMakeFiles/sgm.dir/depend:
	cd /home/finley/CODE/sgm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/finley/CODE/sgm /home/finley/CODE/sgm /home/finley/CODE/sgm/build /home/finley/CODE/sgm/build /home/finley/CODE/sgm/build/CMakeFiles/sgm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sgm.dir/depend

