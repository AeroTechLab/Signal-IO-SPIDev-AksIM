cmake_minimum_required( VERSION 2.8.12 )
project( Signal-IO-SPIDev-AksIM )

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall" )

include( ${CMAKE_CURRENT_LIST_DIR}/interface/CMakeLists.txt )

add_library( SPIDevAksIM MODULE  ${CMAKE_CURRENT_LIST_DIR}/signal_io_spidev.c )
set_target_properties( SPIDevAksIM PROPERTIES PREFIX "" ) 
