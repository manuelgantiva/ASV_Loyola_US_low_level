#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "asv_acados::asv_acados" for configuration ""
set_property(TARGET asv_acados::asv_acados APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(asv_acados::asv_acados PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libasv_acados.so"
  IMPORTED_SONAME_NOCONFIG "libasv_acados.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS asv_acados::asv_acados )
list(APPEND _IMPORT_CHECK_FILES_FOR_asv_acados::asv_acados "${_IMPORT_PREFIX}/lib/libasv_acados.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
