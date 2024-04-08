#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "oal" for configuration "Debug"
set_property(TARGET oal APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(oal PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/liboal.so"
  IMPORTED_SONAME_DEBUG "liboal.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS oal )
list(APPEND _IMPORT_CHECK_FILES_FOR_oal "${_IMPORT_PREFIX}/lib/liboal.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
