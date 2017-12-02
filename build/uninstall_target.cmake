if(NOT EXISTS "/workspace/build/install_manifest.txt")
    message(FATAL_ERROR "Cannot find install manifest: \"/workspace/build/install_manifest.txt\"")
endif(NOT EXISTS "/workspace/build/install_manifest.txt")

file(READ "/workspace/build/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
    message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
    if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        exec_program("/usr/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
            OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
        if(NOT "${rm_retval}" STREQUAL 0)
            message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
        endif(NOT "${rm_retval}" STREQUAL 0)
    else(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
    endif(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
endforeach(file)

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"/workspace/build/install/include/pcl-1.8\"")
if(EXISTS "/workspace/build/install/include/pcl-1.8")
    exec_program("/usr/bin/cmake"
        ARGS "-E remove_directory \"/workspace/build/install/include/pcl-1.8\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"/workspace/build/install/include/pcl-1.8\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "/workspace/build/install/include/pcl-1.8")
    message(STATUS
        "Directory \"/workspace/build/install/include/pcl-1.8\" does not exist.")
endif(EXISTS "/workspace/build/install/include/pcl-1.8")

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
message(STATUS "Uninstalling \"/workspace/build/install/share/pcl-1.8\"")
if(EXISTS "/workspace/build/install/share/pcl-1.8")
    exec_program("/usr/bin/cmake"
        ARGS "-E remove_directory \"/workspace/build/install/share/pcl-1.8\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"/workspace/build/install/share/pcl-1.8\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "/workspace/build/install/share/pcl-1.8")
    message(STATUS
        "Directory \"/workspace/build/install/share/pcl-1.8\" does not exist.")
endif(EXISTS "/workspace/build/install/share/pcl-1.8")

# remove pcl directory in share/doc (removes all files in it!)
if(OFF)
  message(STATUS "Uninstalling \"/workspace/build/install/share/doc/pcl-1.8\"")
  if(EXISTS "/workspace/build/install/share/doc/pcl-1.8")
      exec_program("/usr/bin/cmake"
          ARGS "-E remove_directory \"/workspace/build/install/share/doc/pcl-1.8\""
          OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
      if(NOT "${rm_retval}" STREQUAL 0)
          message(FATAL_ERROR
              "Problem when removing \"/workspace/build/install/share/doc/pcl-1.8\"")
      endif(NOT "${rm_retval}" STREQUAL 0)
  else(EXISTS "/workspace/build/install/share/doc/pcl-1.8")
      message(STATUS
          "Directory \"/workspace/build/install/share/doc/pcl-1.8\" does not exist.")
  endif(EXISTS "/workspace/build/install/share/doc/pcl-1.8")
endif()
