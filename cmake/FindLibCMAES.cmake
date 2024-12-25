# FindLibCMAES.cmake

# Find the LibCMAES includes and library
find_path(LibCMAES_INCLUDE_DIR
  NAMES cmaes.h
  PATH_SUFFIXES libcmaes
  HINTS
    ${LibCMAES_ROOT}
    $ENV{LibCMAES_ROOT}
    /usr/local/include
    /usr/include
)

find_library(LibCMAES_LIBRARY
  NAMES cmaes
  HINTS
    ${LibCMAES_ROOT}
    $ENV{LibCMAES_ROOT}
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibCMAES
  REQUIRED_VARS LibCMAES_LIBRARY LibCMAES_INCLUDE_DIR
)

if(LibCMAES_FOUND)
  set(LibCMAES_INCLUDE_DIRS ${LibCMAES_INCLUDE_DIR})
  set(LibCMAES_LIBRARIES ${LibCMAES_LIBRARY})
  if(NOT TARGET LibCMAES::LibCMAES)
    add_library(LibCMAES::LibCMAES UNKNOWN IMPORTED)
    set_target_properties(LibCMAES::LibCMAES PROPERTIES
      IMPORTED_LOCATION "${LibCMAES_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${LibCMAES_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(LibCMAES_INCLUDE_DIR LibCMAES_LIBRARY)
