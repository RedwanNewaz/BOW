# FindTBB.cmake

find_path(TBB_INCLUDE_DIR
  NAMES tbb/tbb.h
  PATH_SUFFIXES include
  HINTS
    ${TBB_ROOT}
    $ENV{TBB_ROOT}
    /usr/local
    /usr
)

find_library(TBB_LIBRARY
  NAMES tbb
  PATH_SUFFIXES lib lib64
  HINTS
    ${TBB_ROOT}
    $ENV{TBB_ROOT}
    /usr/local
    /usr
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TBB
  REQUIRED_VARS TBB_LIBRARY TBB_INCLUDE_DIR
)

if(TBB_FOUND)
  set(TBB_INCLUDE_DIRS ${TBB_INCLUDE_DIR})
  set(TBB_LIBRARIES ${TBB_LIBRARY})
  if(NOT TARGET TBB::TBB)
    add_library(TBB::TBB UNKNOWN IMPORTED)
    set_target_properties(TBB::TBB PROPERTIES
      IMPORTED_LOCATION "${TBB_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(TBB_INCLUDE_DIR TBB_LIBRARY)
