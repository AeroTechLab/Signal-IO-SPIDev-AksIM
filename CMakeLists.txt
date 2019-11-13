cmake_minimum_required( VERSION 2.8.12 )
project( Signal-IO-SPIDev-AksIM )

set( CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall" )
set( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall" )

include( ${CMAKE_CURRENT_LIST_DIR}/interface/CMakeLists.txt )

add_library( SPIDevAksIM MODULE  ${CMAKE_CURRENT_LIST_DIR}/signal_io_spidev.c )
set_target_properties( SPIDevAksIM PROPERTIES PREFIX "" ) 
