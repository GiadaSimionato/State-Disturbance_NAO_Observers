cmake_minimum_required(VERSION 2.8.12)

project(osgAtlasSimbicon)

find_package(DART 6.1.1 COMPONENTS utils-urdf gui-osg)
find_package(PythonLibs 2.7)
find_package(console_bridge)
find_package(osg)
find_package(osgDB)
find_package(osgUtil)
find_package(osgGA)
find_package(osgViewer)
find_package(assimp)
find_package(OpenThreads)


add_compile_options(-std=c++11)

link_directories(${OSG_LIB_DIR})
link_directories(${ASSIMP_LIBRARY_DIRS})

include_directories(${ASSIMP_INCLUDE_DIRS})
include_directories(${OPENTHREADS_INCLUDE_DIR} ${OSG_INCLUDE_DIR})
include_directories(${DART_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
add_definitions(-DWITHOUT_NUMPY=FALSE)

set(NAO_PLUGIN_BASELIB MPCSolver.cpp qpOASES/BLASReplacement.cpp qpOASES/Bounds.cpp qpOASES/Constraints.cpp qpOASES/Flipper.cpp qpOASES/Indexlist.cpp qpOASES/LAPACKReplacement.cpp qpOASES/Matrices.cpp qpOASES/MessageHandling.cpp qpOASES/Options.cpp qpOASES/OQPinterface.cpp qpOASES/QProblem.cpp qpOASES/QProblemB.cpp qpOASES/SolutionAnalysis.cpp qpOASES/SparseSolver.cpp qpOASES/SQProblem.cpp qpOASES/SQProblemSchur.cpp qpOASES/SubjectTo.cpp qpOASES/Utils.cpp
   )

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs} ${NAO_PLUGIN_BASELIB})

target_link_libraries(${PROJECT_NAME} ${OSG_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGGA_LIBRARIES} ${OPENTHREADS_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME} /usr/local/lib/libassimp.so)

#target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} console_bridge_LIBRARIES)

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()

# print_all_variables()
