
include(UseCython)
include(FindPythonExtensions)
find_package(PythonExtensions REQUIRED)
find_package(Cython REQUIRED)

add_cython_target(geometry CXX)
add_library(geometry MODULE ${geometry})
python_extension_module(geometry)

set_target_properties(geometry
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
