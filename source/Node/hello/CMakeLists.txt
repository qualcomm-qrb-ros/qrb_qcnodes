# Built as part of the qcnode build (see qcnode/source/Node/CMakeLists.txt)
set( QRB_QCNODES_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../../include )
set( QRB_HELLO_HEADERS ${QRB_QCNODES_INCLUDE}/QC/Node/Hello.hpp )

add_library( QCNodeHello OBJECT
    Hello.cpp
    HelloConfig.cpp
    HelloMonitoring.cpp
)

set_property( TARGET QCNodeHello PROPERTY LINKER_LANGUAGE CXX )
set_property( TARGET QCNodeHello PROPERTY POSITION_INDEPENDENT_CODE ON )

target_include_directories( QCNodeHello PUBLIC ${PROJECT_SOURCE_DIR}/include ${QRB_QCNODES_INCLUDE} )
target_link_libraries( QCNodeHello PUBLIC QCNodeBase )
target_compile_options( QCNodeHello PRIVATE -Wall )

install( FILES ${QRB_HELLO_HEADERS} DESTINATION include/QC/Node )

# Add the Hello sample to QCNodeSample so that it can be run by `qcrun ./bin/QCNodeSampleApp -t Hello`.
# QCNodeSample is defined later in the qcnode build (tests/sample), so defer the call. The path is
# expanded here because variables in a deferred call are evaluated at the time it runs.
get_filename_component( QRB_HELLO_SAMPLE ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/sample/SampleHello.cpp ABSOLUTE )
cmake_language( EVAL CODE "cmake_language( DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL target_sources QCNodeSample PRIVATE ${QRB_HELLO_SAMPLE} )" )
