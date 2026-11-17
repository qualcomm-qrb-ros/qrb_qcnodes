# Built as part of the qcnode build (see qcnode/source/Node/CMakeLists.txt)
set( QRB_QCNODES_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../../include )
set( QRB_CONTRACTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../qrb_contracts )

if( NOT TARGET qrb_contracts::qrb_contracts )
    add_subdirectory( ${QRB_CONTRACTS_DIR} qrb_contracts )
endif()

set( QRB_DEMO_HEADERS ${QRB_QCNODES_INCLUDE}/QC/Node/Demo.hpp )

add_library( QCNodeDemo OBJECT
    Demo.cpp
    DemoConfig.cpp
    DemoMonitoring.cpp
)

set_property( TARGET QCNodeDemo PROPERTY LINKER_LANGUAGE CXX )
set_property( TARGET QCNodeDemo PROPERTY POSITION_INDEPENDENT_CODE ON )

target_include_directories( QCNodeDemo PUBLIC ${PROJECT_SOURCE_DIR}/include ${QRB_QCNODES_INCLUDE} )
target_link_libraries( QCNodeDemo PUBLIC QCNodeBase qrb_contracts::qrb_contracts )
target_compile_options( QCNodeDemo PRIVATE -Wall )

install( FILES ${QRB_DEMO_HEADERS} DESTINATION include/QC/Node )

# Add the Demo sample to QCNodeSample so that it can be run by `qcrun ./bin/QCNodeSampleApp -t Demo`.
# QCNodeSample is defined later in the qcnode build (tests/sample), so defer the call. The path is
# expanded here because variables in a deferred call are evaluated at the time it runs.
get_filename_component( QRB_DEMO_SAMPLE ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/sample/SampleDemo.cpp ABSOLUTE )
cmake_language( EVAL CODE "cmake_language( DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL target_sources QCNodeSample PRIVATE ${QRB_DEMO_SAMPLE} )" )
