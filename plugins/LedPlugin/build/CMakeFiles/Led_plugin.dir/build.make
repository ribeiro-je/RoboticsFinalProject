# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build

# Include any dependencies generated for this target.
include CMakeFiles/Led_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Led_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Led_plugin.dir/flags.make

CMakeFiles/Led_plugin.dir/LedPlugin.cc.o: CMakeFiles/Led_plugin.dir/flags.make
CMakeFiles/Led_plugin.dir/LedPlugin.cc.o: ../LedPlugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Led_plugin.dir/LedPlugin.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Led_plugin.dir/LedPlugin.cc.o -c /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/LedPlugin.cc

CMakeFiles/Led_plugin.dir/LedPlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Led_plugin.dir/LedPlugin.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/LedPlugin.cc > CMakeFiles/Led_plugin.dir/LedPlugin.cc.i

CMakeFiles/Led_plugin.dir/LedPlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Led_plugin.dir/LedPlugin.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/LedPlugin.cc -o CMakeFiles/Led_plugin.dir/LedPlugin.cc.s

# Object files for target Led_plugin
Led_plugin_OBJECTS = \
"CMakeFiles/Led_plugin.dir/LedPlugin.cc.o"

# External object files for target Led_plugin
Led_plugin_EXTERNAL_OBJECTS =

libLed_plugin.so: CMakeFiles/Led_plugin.dir/LedPlugin.cc.o
libLed_plugin.so: CMakeFiles/Led_plugin.dir/build.make
libLed_plugin.so: CMakeFiles/Led_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libLed_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Led_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Led_plugin.dir/build: libLed_plugin.so

.PHONY : CMakeFiles/Led_plugin.dir/build

CMakeFiles/Led_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Led_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Led_plugin.dir/clean

CMakeFiles/Led_plugin.dir/depend:
	cd /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build /home/karanr/karan_project/RoboticsFinalProject/plugins/LedPlugin/build/CMakeFiles/Led_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Led_plugin.dir/depend

