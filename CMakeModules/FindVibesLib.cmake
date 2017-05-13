# - Try to find VIBES
# Once done this will define
#  VIBES_FOUND - System has vibes
#  VIBES_INCLUDE_DIRS - The vibes include directories
#  VIBES_LIBRARIES - The libraries needed to use vibes
#  VIBES_DEFINITIONS - Compiler switches required for using vibes

find_package(PkgConfig)
pkg_check_modules(PC_VIBESLIB QUIET vibes)
message(STATUS "VIBES_ROOT ${VIBES_ROOT}")

if(VibesLib_USE_STATIC)
  set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
endif()

set(VIBES_DEFINITIONS ${PC_VIBES_CFLAGS_OTHER})

find_path(VIBES_INCLUDE_DIR vibes.h
          HINTS ${PC_VIBES_INCLUDEDIR} ${PC_VIBES_INCLUDE_DIRS} ${VIBES_ROOT}
          PATH_SUFFIXES include include/vibes )

find_library(VIBES_LIBRARY NAMES vibes-dev
            HINTS ${PC_VIBES_LIBDIR} ${PC_VIBES_LIBRARY_DIRS}  ${VIBES_ROOT}
            PATH_SUFFIXES lib)

set(VIBES_LIBRARIES ${VIBES_LIBRARY})
set(VIBES_INCLUDE_DIRS ${VIBES_INCLUDE_DIR} ${VIBES_INCLUDE_DIR}/..)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set VIBES_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(VIBES  DEFAULT_MSG
                                  VIBES_LIBRARY VIBES_INCLUDE_DIR)

mark_as_advanced(VIBES_INCLUDE_DIR VIBES_LIBRARY )
