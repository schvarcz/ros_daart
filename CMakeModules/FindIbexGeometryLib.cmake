# - Try to find IBEX_GEOMETRY
# Once done this will define
#  IBEX_GEOMETRY_FOUND - System has ibex
#  IBEX_GEOMETRY_INCLUDE_DIRS - The ibex include directories
#  IBEX_GEOMETRY_LIBRARIES - The libraries needed to use ibex
#  IBEX_GEOMETRY_DEFINITIONS - Compiler switches required for using ibex

find_package(PkgConfig)
pkg_check_modules(PC_IBEX_GEOMETRYLIB QUIET ibex)

message(STATUS "IBEX_GEOMETRY_ROOT ${IBEX_GEOMETRY_ROOT}")

if(IbexGeometryLib_USE_STATIC)
  SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
endif()

set(IBEX_GEOMETRY_DEFINITIONS ${PC_IBEX_GEOMETRY_CFLAGS_OTHER})

find_path(IBEX_GEOMETRY_INCLUDE_DIR ibex_SepPolygon.h
          HINTS ${PC_IBEX_GEOMETRY_INCLUDEDIR} ${PC_IBEX_GEOMETRY_INCLUDE_DIRS} ${IBEX_GEOMETRY_ROOT}
          PATH_SUFFIXES include include/ibex-geometry )

find_library(IBEX_GEOMETRY_LIBRARY NAMES ibex-geometry
            HINTS ${PC_IBEX_GEOMETRY_LIBDIR} ${PC_IBEX_GEOMETRY_LIBRARY_DIRS}  ${IBEX_GEOMETRY_ROOT}
            PATH_SUFFIXES lib src
						)

set(IBEX_GEOMETRY_LIBRARIES ${IBEX_GEOMETRY_LIBRARY})
set(IBEX_GEOMETRY_INCLUDE_DIRS ${IBEX_GEOMETRY_INCLUDE_DIR} ${IBEX_GEOMETRY_INCLUDE_DIR}/..)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set IBEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(IBEX_GEOMETRY  DEFAULT_MSG
                                  IBEX_GEOMETRY_LIBRARY IBEX_GEOMETRY_INCLUDE_DIR)

mark_as_advanced(IBEX_GEOMETRY_INCLUDE_DIR IBEX_GEOMETRY_LIBRARY )
